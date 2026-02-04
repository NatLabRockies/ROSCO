! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains additional routines and functions to supplement the primary controllers used in the Controllers module

MODULE ControllerBlocks

USE, INTRINSIC :: ISO_C_Binding
USE Constants
USE Filters
USE Functions
USE SysSubs

IMPLICIT NONE

CONTAINS

    SUBROUTINE PowerControlSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Set up power control
        IF (CntrPar%PRC_Mode == 2) THEN  ! Using power reference control
            IF (CntrPar%PRC_Comm == PRC_Comm_Constant) THEN  ! Constant, from DISCON
                LocalVar%PRC_R_Speed = CntrPar%PRC_R_Speed
                LocalVar%PRC_R_Torque = CntrPar%PRC_R_Torque
                LocalVar%PRC_R_Pitch = CntrPar%PRC_R_Pitch

            ELSEIF (CntrPar%PRC_Comm == PRC_Comm_OpenLoop) THEN  ! Open loop

                IF (CntrPar%Ind_R_Speed > 0) THEN
                    LocalVar%PRC_R_Speed = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_R_Speed,LocalVar%OL_Index,ErrVar)
                    WRITE(401,*) LocalVar%PRC_R_Speed
                ELSE
                    LocalVar%PRC_R_Speed = 1.0_DbKi
                ENDIF

                IF (CntrPar%Ind_R_Torque > 0) THEN
                    LocalVar%PRC_R_Torque = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_R_Torque,LocalVar%OL_Index,ErrVar)
                ELSE
                    LocalVar%PRC_R_Torque = 1.0_DbKi
                ENDIF

                IF (CntrPar%Ind_R_Pitch > 0) THEN
                    LocalVar%PRC_R_Pitch = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_R_Pitch,LocalVar%OL_Index,ErrVar)
                ELSE
                    LocalVar%PRC_R_Pitch = 1.0_DbKi
                ENDIF

            ELSEIF (CntrPar%PRC_Comm == PRC_Comm_ZMQ) THEN  ! ZeroMQ
                LocalVar%PRC_R_Speed    = LocalVar%ZMQ_R_Speed
                LocalVar%PRC_R_Torque   = LocalVar%ZMQ_R_Torque
                LocalVar%PRC_R_Pitch    = LocalVar%ZMQ_R_Pitch

            ENDIF

            ! Set min pitch for power control, will be combined with peak shaving min pitch
            LocalVar%PRC_Min_Pitch = interp1d(CntrPar%PRC_R_Table,CntrPar%PRC_Pitch_Table,LocalVar%PRC_R_Pitch, ErrVar)

        ELSE
            LocalVar%PRC_R_Speed = 1.0_DbKi
            LocalVar%PRC_R_Torque = 1.0_DbKi
            LocalVar%PRC_R_Pitch = 1.0_DbKi
            LocalVar%PRC_Min_Pitch = CntrPar%PC_FinePit
        ENDIF


    END SUBROUTINE

! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE Constants
        ! Allocate variables
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar


        !   Change pitch reference speed
        LocalVar%PC_RefSpd_PRC = CntrPar%PC_RefSpd * LocalVar%PRC_R_Speed
        
        ! Lookup table for speed setpoint (PRC_Mode 1)
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%PRC_WSE_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%PRC_LPF_Freq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
            LocalVar%PC_RefSpd_PRC = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%PRC_WSE_F,ErrVar)
        ENDIF

        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC - LocalVar%SS_DelOmegaF
        ELSE
            LocalVar%PC_RefSpd_SS = LocalVar%PC_RefSpd_PRC
        ENDIF

        ! Compute error for pitch controller
        LocalVar%PC_RefSpd = LocalVar%PC_RefSpd_SS        
        LocalVar%PC_SpdErr = LocalVar%PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error, unused

        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF (CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) THEN
            ! Use unfiltered wind speed estimate, then filter below
            LocalVar%VS_RefSpd_TSR = (CntrPar%VS_TSRopt * LocalVar%WE_Vw / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio

        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) THEN ! Genspeed reference that doesn't depend on wind speed estimate (https://doi.org/10.2172/1259805)
            LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%VS_GenPwr, 0.0)/(CntrPar%VS_GenEff/100.0)/CntrPar%VS_Rgn2K)**(1./3.)

        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR) THEN ! Non-WSE TSR tracking based on square-root of torque
            LocalVar%VS_RefSpd_TSR = (MAX(LocalVar%GenTq, 0.0)/CntrPar%VS_Rgn2K)**(1./2.)

        ELSE ! Generate constant speed reference if K*Omega^2 in use or torque control disabled
            LocalVar%VS_RefSpd_TSR = CntrPar%VS_RefSpd
        ENDIF 

        ! Region 3 FBP reference logic, triggers if Region-2 reference speed is higher than rated
        ! DBS: Alternatively, each of these alternative reference modes could identify Region 3 using their reference-deriving signal, e.g. if WE_Vw > rated speed (accessible in ROSCO?) or GenTq > VS_RtTq
        IF (LocalVar%VS_RefSpd_TSR > CntrPar%VS_RefSpd) THEN
            IF (CntrPar%VS_FBP == VS_FBP_WSE_Ref) THEN ! Use WSE to look up speed reference in Region 3
                LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_U, CntrPar%VS_FBP_Omega, LocalVar%WE_Vw, ErrVar)

            ELSEIF (CntrPar%VS_FBP == VS_FBP_Torque_Ref) THEN ! Use torque to look up speed reference in Region 3
                LocalVar%VS_RefSpd_TSR = interp1d(CntrPar%VS_FBP_Tau, CntrPar%VS_FBP_Omega, LocalVar%GenTq, ErrVar)

            ENDIF
        ENDIF

        ! Change VS Ref speed based on R_Speed
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_TSR * LocalVar%PRC_R_Speed


        ! Filter reference signal
        LocalVar%VS_RefSpd = LPFilter(LocalVar%VS_RefSpd_TSR, LocalVar%DT, CntrPar%F_VSRefSpdCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)


        ! Exclude reference speeds specified by user
        IF (CntrPar%TRA_Mode > 0) THEN
            CALL RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar)
        END IF

        ! Saturate torque reference speed below rated speed if using pitch control in Region 3
        IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
            LocalVar%VS_RefSpd = saturate(LocalVar%VS_RefSpd, CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd * LocalVar%PRC_R_Speed)
        END IF

        ! Simple lookup table for generator speed (PRC_Mode 1)
        IF (CntrPar%PRC_Mode == 1) THEN
            LocalVar%VS_RefSpd = interp1d(CntrPar%PRC_WindSpeeds,CntrPar%PRC_GenSpeeds,LocalVar%PRC_WSE_F,ErrVar)
        ENDIF

        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            LocalVar%VS_RefSpd = LocalVar%VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force minimum rotor speed
        LocalVar%VS_RefSpd = max(LocalVar%VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! Compute speed error from reference
        LocalVar%VS_SpdErr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = LocalVar%VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
        
        ! Region 3 minimum pitch angle for state machine
        LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch

        ! Debug Vars
        DebugVar%VS_RefSpd = LocalVar%VS_RefSpd
        DebugVar%PC_RefSpd = LocalVar%PC_RefSpd


    END SUBROUTINE ComputeVariablesSetpoints
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE StateMachine(CntrPar, LocalVar)
    ! State machine, determines the state of the wind turbine to specify the corresponding control actions
    ! PC States:
    !       PC_State = PC_State_Disabled (0), No pitch control active, BldPitch = PC_MinPit
    !       PC_State = PC_State_Enabled  (1), Active PI blade pitch control enabled
    ! VS States
    !       VS_State = VS_State_Error             (0), Error state, for debugging purposes, GenTq = VS_RtTq
    !       VS_State = VS_State_Region_1_5        (1), Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    !       VS_State = VS_State_Region_2          (2), Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
    !       VS_State = VS_State_Region_2_5        (3), Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    !       VS_State = VS_State_Region_3_ConstTrq (4), above-rated operation using pitch control (constant torque mode)
    !       VS_State = VS_State_Region_3_ConstPwr (5), above-rated operation using pitch and torque control (constant power mode)
    !       VS_State = VS_State_Region_3_FBP      (6), above-rated operation using fixed-pitch torque control
    !       VS_State = VS_State_PI                (7), Tip-Speed-Ratio tracking PI controller
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        
        ! Initialize State machine if first call
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL

            IF (LocalVar%PitCom(1) >= LocalVar%VS_Rgn3Pitch) THEN ! We are in region 3
                LocalVar%PC_State = PC_State_Enabled
                IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN ! Constant power tracking
                    LocalVar%VS_State = VS_State_Region_3_ConstPwr
                ELSE ! Constant torque tracking
                    LocalVar%VS_State = VS_State_Region_3_ConstTrq
                END IF
            ELSE ! We are in Region 2
                LocalVar%VS_State = VS_State_Region_2
                LocalVar%PC_State = PC_State_Disabled
            END IF

        ! Operational States
        ELSE
            ! --- Pitch controller state machine ---
            IF (CntrPar%PC_ControlMode == 1) THEN
                LocalVar%PC_State = PC_State_Enabled
            ELSE 
                LocalVar%PC_State = PC_State_Disabled
            END IF
            
            ! --- Torque control state machine ---
            IF (LocalVar%BlPitchCMeas >= LocalVar%VS_Rgn3Pitch) THEN       
                IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN                   ! Region 3
                    LocalVar%VS_State = VS_State_Region_3_ConstPwr ! Constant power tracking
                ELSE 
                    LocalVar%VS_State = VS_State_Region_3_ConstTrq ! Constant torque tracking
                END IF
            ELSE
                IF (LocalVar%GenArTq >= CntrPar%VS_MaxOMTq*1.01) THEN       ! Region 2 1/2 - active PI torque control
                    IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
                        LocalVar%VS_State = VS_State_Region_2_5
                    ELSE
                        LocalVar%VS_State = VS_State_Region_3_FBP ! Region 3 - fixed blade pitch torque control
                    END IF
                ELSEIF ((LocalVar%GenSpeedF < CntrPar%VS_RefSpd) .AND. &
                        (LocalVar%GenBrTq >= CntrPar%VS_MinOMTq)) THEN      ! Region 2 - optimal torque is proportional to the square of the generator speed
                    LocalVar%VS_State = VS_State_Region_2
                ELSEIF (LocalVar%GenBrTq < CntrPar%VS_MinOMTq) THEN         ! Region 1 1/2
                
                    LocalVar%VS_State = VS_State_Region_1_5
                ELSE                                                        ! Error state, Debug
                    LocalVar%VS_State = VS_State_Error
                END IF
            END IF
        END IF
    END SUBROUTINE StateMachine
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    ! Wind Speed Estimator estimates wind speed at hub height. Currently implements two types of estimators
    !       WE_Mode = 0, Filter hub height wind speed as passed from servodyn using first order low pass filter with 1Hz cornering frequency
    !       WE_Mode = 1, Use Inversion and Inveriance filter as defined by Ortege et. al. 
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, PerformanceData, DebugVariables, ErrorVariables
        USE ieee_arithmetic
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)       :: PerfData
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Allocate Variables
        REAL(DbKi)                 :: F_WECornerFreq   ! Corner frequency (-3dB point) for first order low pass filter for measured hub height wind speed [Hz]

        !       Only used in EKF, if WE_Mode = 2
        REAL(DbKi)                 :: L                ! Turbulent length scale parameter [m]
        REAL(DbKi)                 :: Ti               ! Turbulent intensity, [-]
        ! REAL(DbKi), DIMENSION(3,3) :: I
        !           - operating conditions
        REAL(DbKi)                 :: A_op             ! Estimated operational system pole [UNITS!]
        REAL(DbKi)                 :: Cp_op            ! Estimated operational Cp [-]
        REAL(DbKi)                 :: Tau_r            ! Estimated rotor torque [Nm]
        REAL(DbKi)                 :: a                ! wind variance
        REAL(DbKi)                 :: lambda           ! tip-speed-ratio [rad]
        REAL(DbKi)                 :: RotSpeed         ! Rotor Speed [rad], locally

        !           - Covariance matrices
        REAL(DbKi), DIMENSION(3,3)         :: F        ! First order system jacobian 
        REAL(DbKi), DIMENSION(1,3)         :: H        ! Output equation jacobian 
        REAL(DbKi), DIMENSION(3,1)         :: dxh      ! Estimated state matrix deviation from previous timestep
        REAL(DbKi), DIMENSION(3,3)         :: Q        ! Process noise covariance matrix
        REAL(DbKi), DIMENSION(1,1)         :: S        ! Innovation covariance 
        REAL(DbKi)                         :: R_m      ! Measurement noise covariance [(rad/s)^2]

        REAL(DbKi)              :: WE_Inp_Pitch
        REAL(DbKi)              :: WE_Inp_Torque
        REAL(DbKi)              :: WE_Inp_Speed
        REAL(DbKi)              :: Max_Op_Pitch
        
        CHARACTER(*), PARAMETER                 :: RoutineName = 'WindSpeedEstimator'
        CHARACTER(1024)                        :: WarningMessage

        

        ! Saturate inputs to WSE:
        ! Rotor speed
        IF (LocalVar%RotSpeedF < 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio) THEN
            WE_Inp_Speed = 0.25 * CntrPar%VS_MinOMSpd / CntrPar%WE_GearboxRatio + EPSILON(1.0_DbKi)  ! If this is 0, could cause problems...
        ELSE
            WE_Inp_Speed = LocalVar%RotSpeedF
        END IF

            
        ! Blade pitch
        IF (CntrPar%WE_Mode > 0) THEN ! PerfData is only loaded if WE_Mode > 0
            Max_Op_Pitch = PerfData%Beta_vec(SIZE(PerfData%Beta_vec)) * D2R     ! The Cp surface is only valid up to the end of Beta_vec
        ELSE
            Max_Op_Pitch = 0.0_DbKi    ! Doesn't matter if WE_Mode = 0
        ENDIF
        
        WE_Inp_Pitch = saturate(LocalVar%BlPitchCMeas, CntrPar%PC_MinPit,Max_Op_Pitch) 


        ! Gen torque
        IF (LocalVar%VS_LastGenTrqF < 0.0001 * CntrPar%VS_RtTq) THEN
            WE_Inp_Torque = 0.0001 * CntrPar%VS_RtTq
        ELSE
            WE_Inp_Torque = LocalVar%VS_LastGenTrqF
        END IF

        ! Check to see if in operational range
        LocalVar%WE_Op_Last = LocalVar%WE_Op
        IF (ABS(WE_Inp_Pitch - LocalVar%BlPitchCMeas) > 0) THEN
            LocalVar%WE_Op = 0
        ELSEIF (ABS(WE_Inp_Torque - LocalVar%VS_LastGenTrqF) > 0) THEN
            LocalVar%WE_Op = 0
        ELSEIF (ABS(WE_Inp_Speed - LocalVar%RotSpeedF) > 0) THEN
            LocalVar%WE_Op = 0
        ELSE
            LocalVar%WE_Op = 1
        ENDIF


        ! Restart flag for WSE
        LocalVar%RestartWSE = LocalVar%iStatus      ! Same as iStatus by default

        IF (CntrPar%WE_Mode > 0) THEN
            IF (LocalVar%WE_Op == 0 .AND. LocalVar%WE_Op_Last == 1) THEN   ! Transition from operational to non-operational
                WarningMessage = NewLine//'***************************************************************************************************************************************'//NewLine// &
                    'ROSCO Warning: The wind speed estimator is used, but an input (pitch, rotor speed, or torque) has left the bounds of normal operation.'//NewLine// &
                    'The filtered hub-height wind speed will be used instead. This warning will not persist even though the condition may.'//NewLine// &
                    'Check WE_Op in the ROSCO .dbg file to see if the WSE is enabled (1) or disabled (0).'//NewLine// &
                    '***************************************************************************************************************************************'
                PRINT *, TRIM(WarningMessage)

                LocalVar%RestartWSE = 0 ! Restart
            ENDIF

            IF (LocalVar%WE_Op == 1 .AND. LocalVar%WE_Op_Last == 0) THEN    ! Transition from non-operational to operational
                LocalVar%RestartWSE = 0  ! Restart
            ENDIF
        ENDIF

        ! Filter the wind speed at hub height regardless, only use if WE_Mode = 0 or WE_Op = 0
        ! Re-initialize at WE_Vw if leaving operational wind, WE_Vw is initialized at HorWindV
        LocalVar%HorWindV_F = cos(LocalVar%NacVaneF*D2R) * LPFilter(LocalVar%HorWindV, LocalVar%DT, CntrPar%F_WECornerFreq/10, LocalVar%FP, LocalVar%RestartWSE, LocalVar%restart, objInst%instLPF, LocalVar%WE_Vw)

        ! ---- Debug Inputs ------
        DebugVar%WE_b   = WE_Inp_Pitch
        DebugVar%WE_w   = WE_Inp_Speed
        DebugVar%WE_t   = WE_Inp_Torque

        ! ---- Define wind speed estimate ---- 
        
        ! Inversion and Invariance Filter implementation
        IF (CntrPar%WE_Mode == 1 .AND. LocalVar%WE_Op > 0) THEN      
            ! Compute AeroDynTorque
            Tau_r = AeroDynTorque(LocalVar%RotSpeedF, LocalVar%BlPitchCMeas, LocalVar, CntrPar, PerfData, ErrVar)

            LocalVar%WE_VwIdot = CntrPar%WE_Gamma/CntrPar%WE_Jtot*(LocalVar%VS_LastGenTrq*CntrPar%WE_GearboxRatio - Tau_r)
            LocalVar%WE_VwI = LocalVar%WE_VwI + LocalVar%WE_VwIdot*LocalVar%DT
            LocalVar%WE_Vw = LocalVar%WE_VwI + CntrPar%WE_Gamma*LocalVar%RotSpeedF

        ! Extended Kalman Filter (EKF) implementation
        ELSEIF (CntrPar%WE_Mode == 2 .AND. LocalVar%WE_Op > 0) THEN
            ! Define contant values
            L = 6.0 * CntrPar%WE_BladeRadius
            Ti = 0.18
            R_m = 0.02
            H = RESHAPE((/1.0 , 0.0 , 0.0/),(/1,3/))
            ! Define matrices to be filled
            F = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            Q = RESHAPE((/0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0/),(/3,3/))
            IF (LocalVar%RestartWSE == 0) THEN
                ! Initialize recurring values
                LocalVar%WE%om_r = WE_Inp_Speed
                LocalVar%WE%v_t = 0.0
                LocalVar%WE%v_m = max(LocalVar%HorWindV_F, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV_F is 0, which some AMRWind setups create
                LocalVar%WE%v_h = max(LocalVar%HorWindV_F, 3.0_DbKi)   ! avoid divide by 0 below if HorWindV_F is 0, which some AMRWind setups create
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t   ! Initialize WE_Vw to aviod divide by zero
                lambda = WE_Inp_Speed * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                LocalVar%WE%xh = RESHAPE((/LocalVar%WE%om_r, LocalVar%WE%v_t, LocalVar%WE%v_m/),(/3,1/))
                LocalVar%WE%P = RESHAPE((/0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 1.0/),(/3,3/))
                LocalVar%WE%K = RESHAPE((/0.0,0.0,0.0/),(/3,1/))
                Cp_op   = 0.25  ! initialize so debug output doesn't give *****
                
            ELSE

                ! Find estimated operating Cp and system pole
                A_op = interp1d(CntrPar%WE_FOPoles_v,CntrPar%WE_FOPoles,LocalVar%WE%v_h,ErrVar)

                ! TEST INTERP2D
                lambda = max(WE_Inp_Speed, EPSILON(1.0_DbKi)) * CntrPar%WE_BladeRadius/LocalVar%WE%v_h
                Cp_op = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, WE_Inp_Pitch*R2D, lambda , ErrVar)
                Cp_op = max(0.0,Cp_op)
                
                ! Update Jacobian
                F(1,1) = A_op
                F(1,2) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(1,3) = 1.0/(2.0*CntrPar%WE_Jtot) * CntrPar%WE_RhoAir * PI *CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * Cp_op * LocalVar%WE%v_h**2.0
                F(2,2) = - PI * LocalVar%WE%v_m/(2.0*L)
                F(2,3) = - PI * LocalVar%WE%v_t/(2.0*L)
                
                ! Update process noise covariance
                Q(1,1) = 0.00001
                Q(2,2) =(PI * (LocalVar%WE%v_m**3.0) * (Ti**2.0)) / L
                Q(3,3) = (2.0**2.0)/600.0
                
                ! Prediction update
                Tau_r = AeroDynTorque(WE_Inp_Speed, WE_Inp_Pitch, LocalVar, CntrPar, PerfData, ErrVar)
                a = PI * LocalVar%WE%v_m/(2.0*L)
                dxh(1,1) = 1.0/CntrPar%WE_Jtot * (Tau_r - CntrPar%WE_GearboxRatio * WE_Inp_Torque)
                dxh(2,1) = -a*LocalVar%WE%v_t
                dxh(3,1) = 0.0
                
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%DT * dxh ! state update
                LocalVar%WE%P = LocalVar%WE%P + LocalVar%DT*(MATMUL(F,LocalVar%WE%P) + MATMUL(LocalVar%WE%P,TRANSPOSE(F)) + Q - MATMUL(LocalVar%WE%K * R_m, TRANSPOSE(LocalVar%WE%K))) 
                
                ! Measurement update
                S = MATMUL(H,MATMUL(LocalVar%WE%P,TRANSPOSE(H))) + R_m        ! NJA: (H*T*H') \approx 0
                LocalVar%WE%K = MATMUL(LocalVar%WE%P,TRANSPOSE(H))/S(1,1)
                LocalVar%WE%xh = LocalVar%WE%xh + LocalVar%WE%K*(WE_Inp_Speed - LocalVar%WE%om_r)
                LocalVar%WE%P = MATMUL(identity(3) - MATMUL(LocalVar%WE%K,H),LocalVar%WE%P)
                
                
                ! Wind Speed Estimate
                LocalVar%WE%om_r = max(LocalVar%WE%xh(1,1), EPSILON(1.0_DbKi))
                LocalVar%WE%v_t = LocalVar%WE%xh(2,1)
                LocalVar%WE%v_m = LocalVar%WE%xh(3,1)
                LocalVar%WE%v_h = LocalVar%WE%v_t + LocalVar%WE%v_m
                LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t

                IF (ieee_is_nan(LocalVar%WE%v_h)) THEN
                    LocalVar%WE%om_r = WE_Inp_Speed
                    LocalVar%WE%v_t = 0.0
                    LocalVar%WE%v_m = LocalVar%HorWindV
                    LocalVar%WE%v_h = LocalVar%HorWindV
                    LocalVar%WE_Vw = LocalVar%WE%v_m + LocalVar%WE%v_t
                ENDIF

            ENDIF
            ! Debug Outputs
            DebugVar%WE_Cp = Cp_op
            DebugVar%WE_Vm = LocalVar%WE%v_m
            DebugVar%WE_Vt = LocalVar%WE%v_t
            DebugVar%WE_lambda = lambda
        ELSE        
            ! Use filtered hub-height
            LocalVar%WE_Vw = LocalVar%HorWindV_F
        ENDIF 
        DebugVar%WE_Vw = LocalVar%WE_Vw
        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE WindSpeedEstimator
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE SetpointSmoother(LocalVar, CntrPar, objInst)
    ! Setpoint smoother modifies controller reference in order to separate generator torque and blade pitch control actions
    !       SS_Mode = 0, No setpoint smoothing
    !       SS_Mode = 1, Implement setpoint smoothing
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        ! Allocate Variables
        REAL(DbKi)                      :: DelOmega                            ! Reference generator speed shift, rad/s.
        REAL(DbKi)                      :: R_Total                            ! Total power rating command
        
        ! ------ Setpoint Smoothing ------
        IF ( CntrPar%SS_Mode == 1) THEN
            ! Find setpoint shift amount
            R_Total = LocalVar%PRC_R_Speed * LocalVar%PRC_R_Torque * LocalVar%PRC_R_Pitch
            DelOmega = ((LocalVar%BlPitchCMeas - LocalVar%PC_MinPit)/0.524) * CntrPar%SS_VSGain - ((CntrPar%VS_RtPwr * R_Total - LocalVar%VS_LastGenPwr))/CntrPar%VS_RtPwr * CntrPar%SS_PCGain ! Normalize to 30 degrees for now
            DelOmega = DelOmega * CntrPar%PC_RefSpd
            ! Filter
            LocalVar%SS_DelOmegaF = LPFilter(DelOmega, LocalVar%DT, CntrPar%F_SSCornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF) 
        ELSE
            LocalVar%SS_DelOmegaF = 0 ! No setpoint smoothing
        ENDIF

    END SUBROUTINE SetpointSmoother
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PitchSaturation(LocalVar, CntrPar, objInst, DebugVar, ErrVar) 
    ! PitchSaturation defines a minimum blade pitch angle based on a lookup table provided by DISCON.IN
    ! Minimum pitch for power control also happens here


        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        CHARACTER(*),               PARAMETER           :: RoutineName = 'PitchSaturation'

        ! Define minimum blade pitch angle for peak shaving as a function of estimated wind speed
        LocalVar%PS_Min_Pitch = interp1d(CntrPar%PS_WindSpeeds, CntrPar%PS_BldPitchMin, LocalVar%WE_Vw_F, ErrVar)

        ! Total min pitch limit is greater of peak shaving and power control pitch
        PitchSaturation = max(LocalVar%PS_Min_Pitch, LocalVar%PRC_Min_Pitch)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END FUNCTION PitchSaturation
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Startup(LocalVar, CntrPar, objInst,ErrVar) 
    ! Start up procedure of turbine
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar
        
        ! Local Variables 
        CHARACTER(*),PARAMETER           :: RoutineName = 'Startup'
        Real(DbKi)              :: SU_PrevLoad             ! PRC_R_Toruqe value at the previous stage

        
        ! SU_Stage = -1  (pre startup waiting for transients)
        ! SU_Stage = 1  (freewheeling)
        ! SU_Stage = 2  (load stage 1)
        ! SU_Stage = 3  (load stage 2)
        ! SU_Stage = 4  (load stage 3)
        ! ...
        ! SU_Stage = 0  (startup complete, normal operation)
        
        !Filterd rotor speed
        LocalVar%SU_RotSpeedF = LPFilter(LocalVar%RotSpeed, LocalVar%DT, CntrPar%SU_RotorSpeedCornerFreq, LocalVar%FP,LocalVar%iStatus, LocalVar%restart, objInst%instLPF)

        !Initialize startup stage (SU_Stage)
        IF (LocalVar%iStatus == 0) THEN
            ! Initilize startup stage variable to 1 to denote FreeWheeling
            LocalVar%SU_Stage = -1
        ENDIF

        IF ((LocalVar%SU_Stage == -1) .AND. (LocalVar%Time > CntrPar%SU_StartTime)) THEN
            LocalVar%SU_Stage = 1
        ENDIF
        
        ! Determine last time at which rotor speed was below 0.95*threshold speed during freewheeling
        IF ((LocalVar%SU_Stage == 1) .AND. &
        (LocalVar%SU_RotSpeedF < 0.95_DbKi * CntrPar%SU_RotorSpeedThresh)) THEN
            LocalVar%SU_LoadStageStartTime = LocalVar%Time
        ENDIF
        
        !If free-wheeling exit criteria are met, swtich to load stages
        IF ((LocalVar%SU_Stage == 1) .AND. &
        (LocalVar%Time>=(CntrPar%SU_FW_MinDuration+LocalVar%SU_LoadStageStartTime))) THEN
            LocalVar%SU_LoadStageStartTime = LocalVar%Time
            LocalVar%SU_Stage = 2
        ENDIF

        ! Switch to next load stage when criteria are met
        IF ((LocalVar%SU_Stage .ge. 2) .AND. &
        (LocalVar%Time >= LocalVar%SU_LoadStageStartTime + CntrPar%SU_LoadRampDuration(LocalVar%SU_Stage-1) + &
        CntrPar%SU_LoadHoldDuration(LocalVar%SU_Stage-1)) .AND. &
        (LocalVar%SU_Stage .le. CntrPar%SU_LoadStages_N+1)) THEN
            LocalVar%SU_Stage = LocalVar%SU_Stage + 1
            LocalVar%SU_LoadStageStartTime = LocalVar%Time
        ENDIF

        ! Set PRC_R_Speed, SU_PrevLoad based on SU_Stage
        IF (LocalVar%SU_Stage == 1) THEN
            LocalVar%PRC_R_Speed = CntrPar%SU_RotorSpeedThresh * CntrPar%WE_GearboxRatio / CntrPar%PC_RefSpd
        ELSEIF (LocalVar%SU_Stage == 2) THEN
            SU_PrevLoad = 0.0_DbKi
            ! Ramp up PRC_R_Speed to 1.0 in duration = SU_LoadRampDuration(1)
            LocalVar%PRC_R_Speed = sigma(LocalVar%Time,LocalVar%SU_LoadStageStartTime,    &
                LocalVar%SU_LoadStageStartTime + CntrPar%SU_LoadRampDuration(LocalVar%SU_Stage - 1),    &
                CntrPar%SU_RotorSpeedThresh * CntrPar%WE_GearboxRatio / CntrPar%PC_RefSpd ,1.0_DbKi,ErrVar)
        ELSEIF ((LocalVar%SU_Stage .ge. 2) .AND. (LocalVar%SU_Stage .le. CntrPar%SU_LoadStages_N + 1)) THEN
            SU_PrevLoad = CntrPar%SU_LoadStages(LocalVar%SU_Stage-2)
        ELSEIF (LocalVar%SU_Stage == CntrPar%SU_LoadStages_N + 2) THEN
            ! Set SU_Stage = 0 when startup is over.
            LocalVar%SU_Stage = 0
        ENDIF

        ! Set PRC_R_Torque based on SU_Stage
        IF ((LocalVar%SU_Stage == 1) .OR. (LocalVar%SU_Stage == -1)) THEN
            LocalVar%PRC_R_Torque = 0.0_DbKi
        ELSEIF ((LocalVar%SU_Stage .ge. 2) .AND. (LocalVar%SU_Stage .le. CntrPar%SU_LoadStages_N + 1)) THEN
            IF (LocalVar%Time < LocalVar%SU_LoadStageStartTime + CntrPar%SU_LoadRampDuration(LocalVar%SU_Stage-1)) THEN
                LocalVar%PRC_R_Torque = sigma(LocalVar%Time,LocalVar%SU_LoadStageStartTime,    &
                LocalVar%SU_LoadStageStartTime + CntrPar%SU_LoadRampDuration(LocalVar%SU_Stage - 1),    &
                SU_PrevLoad,CntrPar%SU_LoadStages(LocalVar%SU_Stage - 1),ErrVar)
            ELSE
                LocalVar%PRC_R_Torque = CntrPar%SU_LoadStages(LocalVar%SU_Stage - 1)
            ENDIF
        ENDIF


    END SUBROUTINE Startup
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE Shutdown(LocalVar, CntrPar, objInst,ErrVar) 
    ! Check for shutdown
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar
        
        ! Local Variables 
        CHARACTER(*),               PARAMETER           :: RoutineName = 'Shutdown'
        REAL(DbKi)       :: SD_NacVaneCosF                 ! Time-filtered x-component of NacVane (deg)
        REAL(DbKi)       :: SD_NacVaneSinF                 ! Time-filtered y-component of NacVane (deg)
        INTEGER(IntKi)   :: I_Stage                         ! Loop variable for shutdown stages

        !Initialize shutdown trigger variable
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%SD_Trigger = 0
        ENDIF
        

        ! Filter pitch signal
        LocalVar%SD_BlPitchF = LPFilter(LocalVar%BlPitchCMeas, LocalVar%DT, CntrPar%SD_PitchCornerFreq, LocalVar%FP,LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        ! Filter generator speed
        LocalVar%SD_GenSpeedF = LPFilter(LocalVar%Genspeed, LocalVar%DT, CntrPar%SD_GenSpdCornerFreq, LocalVar%FP,LocalVar%iStatus, LocalVar%restart, objInst%instLPF)

        ! Filter yaw error signal (NacVane)
        SD_NacVaneCosF = LPFilter(cos(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%SD_YawErrorCornerFreq, LocalVar%FP,LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        SD_NacVaneSinF = LPFilter(sin(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%SD_YawErrorCornerFreq, LocalVar%FP,LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
        LocalVar%SD_NacVaneF = wrap_180(atan2(SD_NacVaneSinF, SD_NacVaneCosF) * R2D) ! (in deg)
        
        ! See if we should shutdown
        IF ((LocalVar%SD_Trigger == 0) .AND. (LocalVar%Time>=CntrPar%SD_TimeActivate)) THEN
            IF (CntrPar%SD_EnablePitch==1 .AND. LocalVar%SD_BlPitchF > CntrPar%SD_MaxPit) THEN
                ! Shutdown if above pitch exceeds shutdown threshold
                LocalVar%SD_Trigger  = 1
            ENDIF
            IF (CntrPar%SD_EnableYawError==1 .AND. ABS(LocalVar%SD_NacVaneF) > CntrPar%SD_MaxYawError) THEN
                LocalVar%SD_Trigger = 2
            ENDIF
            IF (CntrPar%SD_EnableGenSpeed==1 .AND. LocalVar%SD_GenSpeedF > CntrPar%SD_MaxGenSpd) THEN
                LocalVar%SD_Trigger = 3
            ENDIF 
            IF (CntrPar%SD_EnableTime==1 .AND. LocalVar%Time > CntrPar%SD_Time) THEN
                LocalVar%SD_Trigger = 4
            ENDIF 
        ENDIF

        ! Method 1: stage depends on time
        IF (CntrPar%SD_Method == 1) THEN
            ! State machine for shutdown stages
            
            IF (LocalVar%SD_Stage == 0) THEN
                ! Normal operation, check for shutdown trigger
                IF (LocalVar%SD_Trigger > 0) THEN
                    LocalVar%SD_Stage = 1 ! First shutdown stage
                    LocalVar%SD_StageStartTime = LocalVar%Time
                ENDIF
                
                ! Set maximum pitch and torque rates (for completeness)
                LocalVar%SD_MaxPitchRate    = 0 
                LocalVar%SD_MaxTorqueRate   = 0 

            ELSEIF (LocalVar%SD_Stage .LE. CntrPar%SD_Stage_N) THEN
                LocalVar%SD_MaxPitchRate    = CntrPar%SD_MaxPitchRate(LocalVar%SD_Stage)
                LocalVar%SD_MaxTorqueRate   = CntrPar%SD_MaxTorqueRate(LocalVar%SD_Stage)
                
                ! Shutdown stage
                IF (LocalVar%Time >= LocalVar%SD_StageStartTime + CntrPar%SD_StageTime(LocalVar%SD_Stage)) THEN
                    LocalVar%SD_Stage = LocalVar%SD_Stage + 1 ! Next shutdown stage
                    LocalVar%SD_StageStartTime = LocalVar%Time
                ENDIF                

            ELSE  ! Stage > CntrPar%SD_Stage_N
                LocalVar%SD_MaxPitchRate    = CntrPar%PC_MaxRat
                LocalVar%SD_MaxTorqueRate   = CntrPar%VS_MaxRat
            ENDIF
       
        ! Shutdown method 2: stage depends on blade pitch (LocalVar%BlPitchCMeas)
        ELSEIF (CntrPar%SD_Method == 2) THEN
            ! State machine for shutdown stages
            
            IF (LocalVar%SD_Stage == 0) THEN
                ! Normal operation, check for shutdown trigger
                IF (LocalVar%SD_Trigger > 0) THEN
                    LocalVar%SD_Stage = 1 ! First shutdown stage
                ENDIF
                
                ! Set maximum pitch and torque rates (for completeness)
                LocalVar%SD_MaxPitchRate    = 0 
                LocalVar%SD_MaxTorqueRate   = 0 

            ELSE ! Stage > 0
                
                ! Figure out what stage we are in
                DO I_Stage = 1, CntrPar%SD_Stage_N
                    IF (LocalVar%BlPitchCMeas >= CntrPar%SD_StagePitch(I_Stage)) THEN
                        LocalVar%SD_Stage = I_Stage + 1
                    ENDIF
                ENDDO
                
                ! Set maximum pitch and torque rates
                IF (LocalVar%SD_Stage > CntrPar%SD_Stage_N) THEN
                    LocalVar%SD_MaxPitchRate    = CntrPar%PC_MaxRat
                    LocalVar%SD_MaxTorqueRate   = CntrPar%VS_MaxRat
                ELSE
                    LocalVar%SD_MaxPitchRate    = CntrPar%SD_MaxPitchRate(LocalVar%SD_Stage)
                    LocalVar%SD_MaxTorqueRate   = CntrPar%SD_MaxTorqueRate(LocalVar%SD_Stage)
                ENDIF 

            ENDIF
        ENDIF



        
        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE Shutdown
!-------------------------------------------------------------------------------------------------------------------------------
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE RefSpeedExclusion(LocalVar, CntrPar, objInst, DebugVar) 
    ! Reference speed exclusion:
    !   Changes torque controllerr reference speed to avoid specified frequencies by a prescribed bandwidth
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters),    INTENT(IN   )       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar 
        TYPE(DebugVariables),      INTENT(INOUT)        :: DebugVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst

        
        REAL(DbKi)                             :: VS_RefSpeed_LSS
        
        ! Get LSS Ref speed
        VS_RefSpeed_LSS = LocalVar%VS_RefSpd/CntrPar%WE_GearboxRatio

        IF ((VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2) .AND. &
            (VS_RefSpeed_LSS < CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2)) THEN
            ! In hysteresis zone, hold reference speed
            LocalVar%FA_Hist = 1 ! Set negative hysteris if ref < exclusion band
        ELSE
            LocalVar%FA_Hist = 0
        END IF

        ! Initialize last reference speed state
        IF (LocalVar%restart) THEN
            ! If starting in hist band
            IF (LocalVar%FA_Hist > 0) THEN
                IF (VS_RefSpeed_LSS > CntrPar%TRA_ExclSpeed) THEN
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed + CntrPar%TRA_ExclBand / 2
                ELSE
                    LocalVar%TRA_LastRefSpd = CntrPar%TRA_ExclSpeed - CntrPar%TRA_ExclBand / 2
                ENDIF
            ELSE
                LocalVar%TRA_LastRefSpd = VS_RefSpeed_LSS
            END IF
        END IF 


        IF (LocalVar%FA_Hist > 0) THEN
            LocalVar%VS_RefSpd_TRA = LocalVar%TRA_LastRefSpd
        ELSE
            LocalVar%VS_RefSpd_TRA = VS_RefSpeed_LSS
        END IF

        ! Save last reference speed       
        LocalVar%TRA_LastRefSpd = LocalVar%VS_RefSpd_TRA

        ! Rate limit reference speed
        LocalVar%VS_RefSpd_RL = ratelimit(LocalVar%VS_RefSpd_TRA, -CntrPar%TRA_RateLimit, CntrPar%TRA_RateLimit, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)
        LocalVar%VS_RefSpd = LocalVar%VS_RefSpd_RL * CntrPar%WE_GearboxRatio


        
    END SUBROUTINE RefSpeedExclusion
!-------------------------------------------------------------------------------------------------------------------------------
END MODULE ControllerBlocks
