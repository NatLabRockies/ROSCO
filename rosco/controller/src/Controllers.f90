! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains the primary controller routines

MODULE Controllers

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE Filters
    USE ControllerBlocks

    IMPLICIT NONE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    ! Blade pitch controller, generally maximizes rotor speed below rated (region 2) and regulates rotor speed above rated (region 3)
    !       PC_State = PC_State_Disabled (0), fix blade pitch to fine pitch angle (PC_FinePit)
    !       PC_State = PC_State_Disabled (1), is gain scheduled PI controller 
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        
        ! Inputs
        REAL(ReKi),              INTENT(INOUT)       :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from the DLL controller.
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        ! Allocate Variables:
        INTEGER(IntKi)                                  :: K            ! Index used for looping through blades.

        CHARACTER(*),               PARAMETER           :: RoutineName = 'PitchControl'

        ! ------- Blade Pitch Controller --------
        ! Load PC State
        IF (LocalVar%PC_State == PC_State_Enabled) THEN ! PI BldPitch control
            LocalVar%PC_MaxPit = CntrPar%PC_MaxPit
        ELSE ! debug mode, fix at fine pitch
            LocalVar%PC_MaxPit = CntrPar%PC_FinePit
        END IF

        ! Hold blade pitch at last value
        ! If:
        !   In pre-startup mode (before freewheeling)
        IF ((CntrPar%SU_Mode > 0) .AND. (LocalVar%SU_Stage == -1)) THEN
            LocalVar%PC_MaxPit = LocalVar%BlPitchCMeas
            LocalVar%PC_MinPit = LocalVar%BlPitchCMeas
        END IF
        
        ! Compute (interpolate) the gains based on previously commanded blade pitch angles and lookup table:
        LocalVar%PC_KP = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KP, LocalVar%BlPitchCMeasF, ErrVar) ! Proportional gain
        LocalVar%PC_KI = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KI, LocalVar%BlPitchCMeasF, ErrVar) ! Integral gain
        LocalVar%PC_KD = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KD, LocalVar%BlPitchCMeasF, ErrVar) ! Derivative gain
        LocalVar%PC_TF = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_TF, LocalVar%BlPitchCMeasF, ErrVar) ! TF gains (derivative filter) !NJA - need to clarify
        
        ! Compute the collective pitch command associated with the proportional and integral gains:
        LocalVar%PC_PitComT = PIController(LocalVar%PC_SpdErr, LocalVar%PC_KP, LocalVar%PC_KI, LocalVar%PC_MinPit, LocalVar%PC_MaxPit, LocalVar%DT, LocalVar%BlPitch(1), LocalVar%piP, LocalVar%restart, objInst%instPI)
        DebugVar%PC_PICommand = LocalVar%PC_PitComT

        ! Find individual pitch control contribution
        IF ((CntrPar%IPC_ControlMode >= 1) .OR. (CntrPar%Y_ControlMode == 2)) THEN
            CALL IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        ELSE
            LocalVar%IPC_PitComF = 0.0 ! THIS IS AN ARRAY!!
        END IF
        
        ! Include tower fore-aft tower vibration damping control
        IF (CntrPar%TD_Mode > 0) THEN
            CALL ForeAftDamping(CntrPar, LocalVar, objInst)
        ELSE
            LocalVar%FA_PitCom = 0.0 ! THIS IS AN ARRAY!!
        ENDIF
        
        ! Pitch Saturation
        IF (CntrPar%PS_Mode > 0) THEN
            LocalVar%PC_MinPit = PitchSaturation(LocalVar,CntrPar,objInst,DebugVar, ErrVar)
            LocalVar%PC_MinPit = max(LocalVar%PC_MinPit, CntrPar%PC_FinePit)
        ELSE
            LocalVar%PC_MinPit = CntrPar%PC_FinePit
        ENDIF
        DebugVar%PC_MinPit = LocalVar%PC_MinPit
        
        ! FloatingFeedback
        IF (CntrPar%Fl_Mode > 0) THEN
            LocalVar%Fl_PitCom = FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar)
            DebugVar%FL_PitCom = LocalVar%Fl_PitCom
            LocalVar%PC_PitComT = LocalVar%PC_PitComT + LocalVar%Fl_PitCom
        ENDIF
        
        ! Saturate collective pitch commands:
        LocalVar%PC_PitComT = saturate(LocalVar%PC_PitComT, LocalVar%PC_MinPit, CntrPar%PC_MaxPit)                    ! Saturate the overall command using the pitch angle limits
        LocalVar%PC_PitComT = ratelimit(LocalVar%PC_PitComT, CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitchCMeas) ! Saturate the overall command of blade K using the pitch rate limit
        LocalVar%PC_PitComT_Last = LocalVar%PC_PitComT

        ! Combine and saturate all individual pitch commands in software
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            LocalVar%PitCom(K) = LocalVar%PC_PitComT + LocalVar%FA_PitCom(K) 
            LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K), LocalVar%PC_MinPit, CntrPar%PC_MaxPit)                    ! Saturate the command using the pitch saturation limits
            LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%IPC_PitComF(K)                                          ! Add IPC
            
            ! Hard IPC saturation by peak shaving limit
            IF (CntrPar%IPC_SatMode == 1) THEN
                LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K), LocalVar%PC_MinPit, CntrPar%PC_MaxPit)  
            END IF
            
            ! Add ZeroMQ pitch commands
            LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%ZMQ_PitOffset(K)

            ! Rate limit                  
            LocalVar%PitCom(K) = ratelimit(LocalVar%PitCom(K), CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitch(K)) ! Saturate the overall command of blade K using the pitch rate limit
        END DO 

        ! Open Loop control, use if
        !   Open loop mode active         Using OL blade pitch control      
        IF (CntrPar%OL_Mode > 0) THEN
            IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN    ! Time > first open loop breakpoint
                IF (CntrPar%Ind_BldPitch(1) > 0) THEN
                    LocalVar%PitCom(1) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch1,LocalVar%OL_Index, ErrVar)
                ENDIF

                IF (CntrPar%Ind_BldPitch(2) > 0) THEN
                    LocalVar%PitCom(2) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch2,LocalVar%OL_Index, ErrVar)
                ENDIF

                IF (CntrPar%Ind_BldPitch(3) > 0) THEN
                    LocalVar%PitCom(3) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_BldPitch3,LocalVar%OL_Index, ErrVar)
                ENDIF
            ENDIF
        ENDIF

        ! Active wake control
        IF (CntrPar%AWC_Mode > 0) THEN
            CALL ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        ENDIF

        ! Shutdown
        IF (LocalVar%SD_Trigger == 0) THEN
            LocalVar%PitCom_SD = LocalVar%PitCom
        ! If shutdown is not triggered, PitCom_SD tracks PitCom.
        ELSE
            IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
                DO K = 1,LocalVar%NumBl
                    LocalVar%PitCom_SD(K) = LocalVar%PitCom_SD(K) + LocalVar%SD_MaxPitchRate*LocalVar%DT
                END DO
            ENDIF
            LocalVar%PitCom = LocalVar%PitCom_SD
            ! When shutdown is triggered (SD_Trigger \=0), pitch to feather.
            ! Note that in some instances (like a downwind rotor), we may want to pitch to a stall angle.
        ENDIF
        
       
        ! Place pitch actuator here, so it can be used with or without open-loop
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            IF (CntrPar%PA_Mode > 0) THEN
                IF (CntrPar%PA_Mode == 1) THEN
                    LocalVar%PitComAct(K) = LPFilter(LocalVar%PitCom(K), LocalVar%DT, CntrPar%PA_CornerFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
                ELSE IF (CntrPar%PA_Mode == 2) THEN
                    LocalVar%PitComAct(K) = SecLPFilter(LocalVar%PitCom(K),LocalVar%DT,CntrPar%PA_CornerFreq,CntrPar%PA_Damping,LocalVar%FP,LocalVar%iStatus,LocalVar%restart,objInst%instSecLPF)
                END IF  
            ELSE
                LocalVar%PitComAct(K) = LocalVar%PitCom(K)
            ENDIF
        END DO

        ! Hardware saturation: using CntrPar%PC_MinPit
        DO K = 1,LocalVar%NumBl ! Loop through all blades, add IPC contribution and limit pitch rate
            ! Saturate the pitch command using the overall (hardware) limit
            LocalVar%PitComAct(K) = saturate(LocalVar%PitComAct(K), CntrPar%PC_MinPit, CntrPar%PC_MaxPit)
            ! Saturate the overall command of blade K using the pitch rate limit
            LocalVar%PitComAct(K) = ratelimit(LocalVar%PitComAct(K), CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL,LocalVar%BlPitch(K)) ! Saturate the overall command of blade K using the pitch rate limit
        END DO

        ! Add pitch actuator fault for blade K
        IF (CntrPar%PF_Mode == 1) THEN
            DO K = 1, LocalVar%NumBl
                ! This assumes that the pitch actuator fault overides the Hardware saturation
                LocalVar%PitComAct(K) = LocalVar%PitComAct(K) + CntrPar%PF_Offsets(K)
            END DO
        ! Blade pitch stuck at last value
        ELSEIF (CntrPar%PF_Mode == 2) THEN
            DO K = 1, LocalVar%NumBl
                IF (LocalVar%Time > CntrPar%PF_TimeStuck(K)) THEN
                    LocalVar%PitComAct(K) = LocalVar%BlPitch(K)
                END IF
            END DO
        END IF

        ! Command the pitch demanded from the last
        ! call to the controller (See Appendix A of Bladed User's Guide):
        avrSWAP(42) = LocalVar%PitComAct(1)   ! Use the command angles of all blades if using individual pitch
        avrSWAP(43) = LocalVar%PitComAct(2)   ! "
        avrSWAP(44) = LocalVar%PitComAct(3)   ! "
        avrSWAP(45) = LocalVar%PitComAct(1)   ! Use the command angle of blade 1 if using collective pitch

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
    END SUBROUTINE PitchControl
!-------------------------------------------------------------------------------------------------------------------------------  
    SUBROUTINE VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
    ! Generator torque controller
    !       VS_State = VS_State_Error             (0), Error state, for debugging purposes, GenTq = VS_RtTq
    !       VS_State = VS_State_Region_1_5        (1), Region 1(.5) operation, torque control to keep the rotor at cut-in speed towards the Cp-max operational curve
    !       VS_State = VS_State_Region_2          (2), Region 2 operation, maximum rotor power efficiency (Cp-max) tracking using K*omega^2 law, fixed fine-pitch angle in BldPitch controller
    !       VS_State = VS_State_Region_2_5        (3), Region 2.5, transition between below and above-rated operating conditions (near-rated region) using PI torque control
    !       VS_State = VS_State_Region_3_ConstTrq (4), above-rated operation using pitch control (constant torque mode)
    !       VS_State = VS_State_Region_3_ConstPwr (5), above-rated operation using pitch and torque control (constant power mode)
    !       VS_State = VS_State_PI                (6), Tip-Speed-Ratio tracking PI controller (ignore state machine)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        ! Inputs
        REAL(ReKi),                 INTENT(INOUT)       :: avrSWAP(*)    ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar

        CHARACTER(*),               PARAMETER           :: RoutineName = 'VariableSpeedControl'

        ! Allocate Variables

        ! -------- Variable-Speed Torque Controller --------

        ! Pre-compute generator torque values for K*Omega^2 and constant power
        LocalVar%VS_KOmega2_GenTq = CntrPar%VS_Rgn2K*LocalVar%GenSpeedF*LocalVar%GenSpeedF
        LocalVar%VS_ConstPwr_GenTq = (CntrPar%VS_RtPwr/(CntrPar%VS_GenEff/100.0))/LocalVar%GenSpeedF * LocalVar%PRC_R_Torque

        ! Determine maximum torque saturation limit, VS_MaxTq
        IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN 
            ! Variable pitch mode        
            IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN
                LocalVar%VS_MaxTq = min(LocalVar%VS_ConstPwr_GenTq, CntrPar%VS_MaxTq)
            ELSE
                LocalVar%VS_MaxTq = CntrPar%VS_RtTq * LocalVar%PRC_R_Torque
            END IF
        ELSE   
            ! Constant pitch, max torque is control parameter, de-rated by the torque rating
            LocalVar%VS_MaxTq = CntrPar%VS_MaxTq * LocalVar%PRC_R_Torque
        ENDIF 

        ! Optimal Tip-Speed-Ratio tracking controller (reference generated in subroutine ComputeVariablesSetpoints)
        IF ((CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) .OR. \
            (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) .OR. \
            (CntrPar%VS_ControlMode == VS_Mode_Torque_TSR)) THEN


            ! PI controller
            LocalVar%GenTq = PIController( &
                                        LocalVar%VS_SpdErr, &
                                        CntrPar%VS_KP(1), &
                                        CntrPar%VS_KI(1), &
                                        CntrPar%VS_MinTq, LocalVar%VS_MaxTq, &
                                        LocalVar%DT, LocalVar%VS_LastGenTrq, LocalVar%piP, LocalVar%restart, objInst%instPI)

            ! Saturate control input to Region 3 constant-power value if FBP mode is set to constant-power overspeed (no need for explicit transition region)
            IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
                LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%GenTq)
            ENDIF

        ! K*Omega^2 control law with PI torque control in transition regions
        ELSEIF (CntrPar%VS_ControlMode == VS_Mode_KOmega) THEN
            ! Update PI loops for region 1.5 and 2.5 PI control
            LocalVar%GenArTq = PIController(LocalVar%VS_SpdErrAr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MaxOMTq, CntrPar%VS_ArSatTq, LocalVar%DT, CntrPar%VS_MaxOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)
            LocalVar%GenBrTq = PIController(LocalVar%VS_SpdErrBr, CntrPar%VS_KP(1), CntrPar%VS_KI(1), CntrPar%VS_MinTq, CntrPar%VS_MinOMTq, LocalVar%DT, CntrPar%VS_MinOMTq, LocalVar%piP, LocalVar%restart, objInst%instPI)

            ! State machine if switching to blade pitch control
            IF (LocalVar%VS_State == VS_State_Region_1_5) THEN ! Region 1.5
                LocalVar%GenTq = LocalVar%GenBrTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_2) THEN ! Region 2
                LocalVar%GenTq = LocalVar%VS_KOmega2_GenTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_2_5) THEN ! Region 2.5
                LocalVar%GenTq = LocalVar%GenArTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstTrq) THEN ! Region 3, constant torque
                LocalVar%GenTq = CntrPar%VS_RtTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_ConstPwr) THEN ! Region 3, constant power
                LocalVar%GenTq = LocalVar%VS_ConstPwr_GenTq
            ELSEIF (LocalVar%VS_State == VS_State_Region_3_FBP) THEN ! Region 3, fixed blade pitch
                ! Constant power overspeed
                IF (CntrPar%VS_FBP == VS_FBP_Power_Overspeed) THEN
                    ! K*Omega^2 in Region 2 or constant power overspeed in Region 3
                    LocalVar%GenTq = MIN(LocalVar%VS_ConstPwr_GenTq, LocalVar%VS_KOmega2_GenTq)
                ! Reference-tracking in Region 3
                ELSEIF ((CntrPar%VS_FBP == VS_FBP_WSE_Ref) .OR. (CntrPar%VS_FBP == VS_FBP_Torque_Ref)) THEN
                    LocalVar%GenTq = LocalVar%GenArTq
                ENDIF
            END IF

        ELSE        ! VS_ControlMode of 0
            LocalVar%GenTq = 0
        ENDIF
        
        
        ! Shutdown
        IF (LocalVar%SD_Trigger == 0) THEN
            LocalVar%GenTq_SD = LocalVar%GenTq
        ELSE 
            IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
                LocalVar%GenTq_SD = LocalVar%GenTq_SD - LocalVar%SD_MaxTorqueRate*LocalVar%DT
                LocalVar%GenTq_SD = saturate(LocalVar%GenTq_SD, CntrPar%VS_MinTq, CntrPar%VS_MaxTq)
            ENDIF
            LocalVar%GenTq = LocalVar%GenTq_SD
        ENDIF

        ! Saturate based on most stringent defined maximum
        LocalVar%GenTq = saturate(LocalVar%GenTq, CntrPar%VS_MinTq, MIN(CntrPar%VS_MaxTq, LocalVar%VS_MaxTq))

        ! Saturate the commanded torque using the torque rate limit
        LocalVar%GenTq = ratelimit(LocalVar%GenTq, -CntrPar%VS_MaxRat, CntrPar%VS_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,objInst%instRL)    ! Saturate the command using the torque rate limit

        ! Open loop torque control
        IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0)) THEN
            ! Get current OL GenTq, applies for OL_Mode 1 and 2
            IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
                LocalVar%GenTq = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_GenTq,LocalVar%OL_Index,ErrVar)
            ENDIF
            
            ! Azimuth tracking control
            IF (CntrPar%OL_Mode == 2) THEN
                
                ! Push, pop and unwrap azimuth buffer 
                ! Initialize
                IF (LocalVar%iStatus == 0) THEN
                    LocalVar%AzBuffer(1) = LocalVar%Azimuth
                    LocalVar%AzBuffer(2) = LocalVar%Azimuth
                ENDIF
                LocalVar%AzBuffer(1) = LocalVar%AzBuffer(2)
                LocalVar%AzBuffer(2) = LocalVar%Azimuth
                LocalVar%AzBuffer = UNWRAP(LocalVar%AzBuffer, ErrVar)
                LocalVar%AzUnwrapped = LocalVar%AzBuffer(2)

                ! Current desired Azimuth, error
                LocalVar%OL_Azimuth = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_Azimuth,LocalVar%Time,ErrVar)
                LocalVar%AzError = LocalVar%OL_Azimuth - LocalVar%AzUnwrapped 

                LocalVar%GenTqAz = PIDController(LocalVar%AzError, CntrPar%RP_Gains(1), CntrPar%RP_Gains(2), CntrPar%RP_Gains(3), CntrPar%RP_Gains(4), -LocalVar%VS_MaxTq * 2, LocalVar%VS_MaxTq * 2, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst, LocalVar)
                LocalVar%GenTq = LocalVar%GenTq + LocalVar%GenTqAz

            ENDIF

        ENDIF

        ! Reset the value of LocalVar%VS_LastGenTrq to the current values:
        LocalVar%VS_LastGenTrq = LocalVar%GenTq
        LocalVar%VS_LastGenPwr = LocalVar%VS_GenPwr
        
        ! Set the command generator torque (See Appendix A of Bladed User's Guide):
        avrSWAP(47) = MAX(0.0_DbKi, LocalVar%VS_LastGenTrq)  ! Demanded generator torque, prevent negatives.

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE VariableSpeedControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        ! Yaw rate controller
        !       Y_ControlMode = 0, No yaw control
        !       Y_ControlMode = 1, Yaw rate control using yaw drive

        ! TODO: Lots of R2D->D2R, this should be cleaned up.
        ! TODO: The constant offset implementation is sort of circular here as a setpoint is already being defined in SetVariablesSetpoints. This could also use cleanup
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
    
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*) ! The swap array, used to pass data to, and receive data from, the DLL controller.
    
        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        TYPE(DebugVariables), INTENT(INOUT)       :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT)       :: ErrVar

        ! Allocate Variables
        REAL(DbKi), SAVE :: NacVaneOffset                          ! For offset control
        INTEGER, SAVE :: YawState                               ! Yawing left(-1), right(1), or stopped(0)
        REAL(DbKi)       :: WindDirPlusOffset                     ! Instantaneous wind direction minus the assigned vane offset (deg)
        REAL(DbKi)       :: WindDirPlusOffsetCosF                 ! Time-filtered x-component of WindDirPlusOffset (deg)
        REAL(DbKi)       :: WindDirPlusOffsetSinF                 ! Time-filtered y-component of WindDirPlusOffset (deg)
        REAL(DbKi)       :: NacHeadingTarget                       ! Time-filtered wind direction minus the assigned vane offset (deg)
        REAL(DbKi), SAVE :: NacHeadingError                        ! Yaw error (deg)
        REAL(DbKi)       :: YawRateCom                             ! Commanded yaw rate (deg/s)
        REAL(DbKi)       :: deadband                               ! Allowable yaw error deadband (deg)
        REAL(DbKi)       :: Time                                   ! Current time
        INTEGER, SAVE :: Tidx                                   ! Index i: commanded yaw error is interpolated between i and i+1
        
        IF (CntrPar%Y_ControlMode == 1) THEN

            ! Compass wind directions in degrees
            LocalVar%WindDir = wrap_180(LocalVar%NacHeading + LocalVar%NacVane)

            ! Initialize
            IF (LocalVar%iStatus == 0) THEN
                YawState = 0
                Tidx = 1
            ENDIF
            
            ! Compute/apply offset
            IF (CntrPar%ZMQ_Mode == 1) THEN
                NacVaneOffset = LocalVar%ZMQ_YawOffset
            ELSE
                NacVaneOffset = CntrPar%Y_MErrSet ! (deg) # Offset from setpoint
            ENDIF

            ! Update filtered wind direction
            WindDirPlusOffset = wrap_180(LocalVar%WindDir + NacVaneOffset) ! (deg)
            WindDirPlusOffsetCosF = LPFilter(cos(WindDirPlusOffset*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
            WindDirPlusOffsetSinF = LPFilter(sin(WindDirPlusOffset*D2R), LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
            NacHeadingTarget = wrap_180(atan2(WindDirPlusOffsetSinF, WindDirPlusOffsetCosF) * R2D) ! (deg)

            ! ---- Now get into the guts of the control ----
            ! Yaw error
            NacHeadingError = wrap_180(NacHeadingTarget - LocalVar%NacHeading)
			
            ! Check for deadband
            IF (LocalVar%WE_Vw_F .le. CntrPar%Y_uSwitch) THEN
                deadband = CntrPar%Y_ErrThresh(1)
            ELSE
                deadband = CntrPar%Y_ErrThresh(2)
            ENDIF

            ! yawing right
            IF (YawState == 1) THEN 
                IF (NacHeadingError .le. 0) THEN
                    ! stop yawing
                    YawRateCom = 0.0
                    YawState = 0 
                ELSE
                    ! persist
                    YawRateCom = CntrPar%Y_Rate
                    YawState = 1 
                ENDIF
            ! yawing left
            ELSEIF (YawState == -1) THEN 
                IF (NacHeadingError .ge. 0) THEN
                    ! stop yawing
                    YawRateCom = 0.0
                    YawState = 0 
                ELSE
                    ! persist
                    YawRateCom = -CntrPar%Y_Rate
                    YawState = -1 
                ENDIF
            ! Initiate yaw if outside yaw error threshold
            ELSE
                IF (NacHeadingError .gt. deadband) THEN
                    YawState = 1 ! yaw right
                ENDIF

                IF (NacHeadingError .lt. -deadband) THEN
                    YawState = -1 ! yaw left
                ENDIF

                YawRateCom = 0.0 ! if YawState is not 0, start yawing on the next time step
            ENDIF

            ! Output yaw rate command in rad/s
            avrSWAP(48) = YawRateCom * D2R

            ! If using open loop yaw rate control, overwrite controlled output
            ! Open loop yaw rate control - control input in rad/s
            IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_YawRate > 0)) THEN
                IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
                    avrSWAP(48) = interp1d(CntrPar%OL_Breakpoints,CntrPar%OL_YawRate,LocalVar%OL_Index, ErrVar)
                ENDIF
            ENDIF

            ! Save for debug
            DebugVar%YawRateCom       = YawRateCom
            DebugVar%NacHeadingTarget = NacHeadingTarget
            DebugVar%NacVaneOffset    = NacVaneOffset
            DebugVar%YawState         = YawState
            DebugVar%Yaw_Err          = NacHeadingError
        END IF
    END SUBROUTINE YawRateControl
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        ! Individual pitch control subroutine
        !   - Calculates the commanded pitch angles for IPC employed for blade fatigue load reductions at 1P and 2P
        !   - Includes yaw by IPC

        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)       :: objInst
        TYPE(DebugVariables),       INTENT(INOUT)        :: DebugVar
        TYPE(ErrorVariables),       INTENT(INOUT)        :: ErrVar

        ! Local variables
        REAL(DbKi)                  :: PitComIPC(3), PitComIPCF(3), PitComIPC_1P(3), PitComIPC_2P(3)
        INTEGER(IntKi)              :: i, K                                    ! Integer used to loop through gains and turbine blades
        REAL(DbKi)                  :: axisYawIPC_1P                           ! IPC contribution with yaw-by-IPC component
        REAL(DbKi)                  :: Y_MErr, Y_MErrF, Y_MErrF_IPC            ! Unfiltered and filtered yaw alignment error [rad]
        
        CHARACTER(*),               PARAMETER           :: RoutineName = 'IPC'

        ! Body
        ! Pass rootMOOPs through the Coleman transform to get the tilt and yaw moment axis
        CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, NP_1, LocalVar%axisTilt_1P, LocalVar%axisYaw_1P)
        CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, NP_2, LocalVar%axisTilt_2P, LocalVar%axisYaw_2P)

        ! High-pass filter the MBC yaw component and filter yaw alignment error, and compute the yaw-by-IPC contribution
        IF (CntrPar%Y_ControlMode == 2) THEN
            Y_MErr = wrap_360(LocalVar%NacHeading + LocalVar%NacVane)
            Y_MErrF = LPFilter(Y_MErr, LocalVar%DT, CntrPar%F_YawErr, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPF)
            Y_MErrF_IPC = PIController(Y_MErrF, CntrPar%Y_IPC_KP, CntrPar%Y_IPC_KI, -CntrPar%Y_IPC_IntSat, CntrPar%Y_IPC_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
        ELSE
            LocalVar%axisYawF_1P = LocalVar%axisYaw_1P
            Y_MErrF = 0.0
            Y_MErrF_IPC = 0.0
        END IF

        ! Soft cutin with sigma function 
        DO i = 1,2
            LocalVar%IPC_KP(i) = sigma(LocalVar%WE_Vw, CntrPar%IPC_Vramp(1), CntrPar%IPC_Vramp(2), 0.0_DbKi, CntrPar%IPC_KP(i), ErrVar)
            LocalVar%IPC_KI(i) = sigma(LocalVar%WE_Vw, CntrPar%IPC_Vramp(1), CntrPar%IPC_Vramp(2), 0.0_DbKi, CntrPar%IPC_KI(i), ErrVar)
        END DO

        ! Handle saturation limit, depends on IPC_SatMode
        IF (CntrPar%IPC_SatMode == 2) THEN
            ! Saturate to min allowed pitch angle, softly using IPC_IntSat
            LocalVar%IPC_IntSat = min(CntrPar%IPC_IntSat,LocalVar%BlPitchCMeas - CntrPar%PC_MinPit)
        ELSEIF (CntrPar%IPC_SatMode == 3) THEN
            ! Saturate to peak shaving, softly using IPC_IntSat
            LocalVar%IPC_IntSat = min(CntrPar%IPC_IntSat,LocalVar%BlPitchCMeas - LocalVar%PC_MinPit)
        ELSE
            LocalVar%IPC_IntSat = CntrPar%IPC_IntSat
        ENDIF
        
        ! Integrate the signal and multiply with the IPC gain
        IF (CntrPar%IPC_ControlMode >= 1 .AND. CntrPar%Y_ControlMode /= 2)  THEN
            LocalVar%IPC_axisTilt_1P = PIController(LocalVar%axisTilt_1P, LocalVar%IPC_KP(1), LocalVar%IPC_KI(1), -LocalVar%IPC_IntSat, LocalVar%IPC_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
            LocalVar%IPC_axisYaw_1P = PIController(LocalVar%axisYawF_1P, LocalVar%IPC_KP(1), LocalVar%IPC_KI(1), -LocalVar%IPC_IntSat, LocalVar%IPC_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
            
            IF (CntrPar%IPC_ControlMode >= 2) THEN
                LocalVar%IPC_axisTilt_2P = PIController(LocalVar%axisTilt_2P, LocalVar%IPC_KP(2), LocalVar%IPC_KI(2), -LocalVar%IPC_IntSat, LocalVar%IPC_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
                LocalVar%IPC_axisYaw_2P = PIController(LocalVar%axisYawF_2P, LocalVar%IPC_KP(2), LocalVar%IPC_KI(2), -LocalVar%IPC_IntSat, LocalVar%IPC_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
            END IF
        ELSE
            LocalVar%IPC_axisTilt_1P = 0.0
            LocalVar%IPC_axisYaw_1P = 0.0
            LocalVar%IPC_axisTilt_2P = 0.0
            LocalVar%IPC_axisYaw_2P = 0.0
        ENDIF
        
        ! Add the yaw-by-IPC contribution
        axisYawIPC_1P = LocalVar%IPC_axisYaw_1P + Y_MErrF_IPC
        
        ! Pass direct and quadrature axis through the inverse Coleman transform to get the commanded pitch angles
        CALL ColemanTransformInverse(LocalVar%IPC_axisTilt_1P, axisYawIPC_1P, LocalVar%Azimuth, NP_1, CntrPar%IPC_aziOffset(1), PitComIPC_1P)
        CALL ColemanTransformInverse(LocalVar%IPC_axisTilt_2P, LocalVar%IPC_axisYaw_2P, LocalVar%Azimuth, NP_2, CntrPar%IPC_aziOffset(2), PitComIPC_2P)
        
        ! Sum nP IPC contributions and store to LocalVar data type
        DO K = 1,LocalVar%NumBl
            PitComIPC(K) = PitComIPC_1P(K) + PitComIPC_2P(K)
            
            ! Optionally filter the resulting signal to induce a phase delay
            IF (CntrPar%IPC_CornerFreqAct > 0.0) THEN
                PitComIPCF(K) = LPFilter(PitComIPC(K), LocalVar%DT, CntrPar%IPC_CornerFreqAct, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            ELSE
                PitComIPCF(K) = PitComIPC(K)
            END IF
            
            LocalVar%IPC_PitComF(K) = PitComIPCF(K)
        END DO


        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE IPC
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
        ! Fore-aft damping controller, reducing the tower fore-aft vibrations using pitch

        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        
        ! Local variables
        INTEGER(IntKi) :: K    ! Integer used to loop through turbine blades

        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        
        ! Body
        LocalVar%FA_AccHPFI = PIController(LocalVar%FA_AccHPF, 0.0_DbKi, CntrPar%FA_KI, -CntrPar%FA_IntSat, CntrPar%FA_IntSat, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
        
        ! Store the fore-aft pitch contribution to LocalVar data type
        DO K = 1,LocalVar%NumBl
            LocalVar%FA_PitCom(K) = LocalVar%FA_AccHPFI
        END DO
        
    END SUBROUTINE ForeAftDamping
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar) 
    ! FloatingFeedback defines a minimum blade pitch angle based on a lookup table provided by DISON.IN
    !       Fl_Mode = 0, No feedback
    !       Fl_Mode = 1, Proportional feedback of nacelle velocity (translational)
    !       Fl_Mode = 2, Proportional feedback of nacelle velocity (rotational)
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, ErrorVariables
        IMPLICIT NONE
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar 
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst
        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar
        ! Allocate Variables 
        REAL(DbKi)                      :: FA_vel ! Tower fore-aft velocity [m/s]
        REAL(DbKi)                      :: NacIMU_FA_vel ! Tower fore-aft pitching velocity [rad/s]

        ! Gain scheduling
        LocalVar%Kp_Float = interp1d(CntrPar%Fl_U,CntrPar%Fl_Kp,LocalVar%WE_Vw_F,ErrVar)       ! Schedule based on WSE
        
        ! Calculate floating contribution to pitch command
        FA_vel = PIController(LocalVar%FA_AccF, 0.0_DbKi, 1.0_DbKi, -100.0_DbKi , 100.0_DbKi ,LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) ! NJA: should never reach saturation limits....
        NacIMU_FA_vel = PIController(LocalVar%NacIMU_FA_AccF, 0.0_DbKi, 1.0_DbKi, -100.0_DbKi , 100.0_DbKi ,LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) ! NJA: should never reach saturation limits....        
! Mod made by A. Wright: use the gain scheduled value of KPfloat in the floating fb equ's below (instead of the old value of CntrPar%Fl_Kp), for either value of CntrPar%Fl_Mode...        
        if (CntrPar%Fl_Mode == 1) THEN
            FloatingFeedback = (0.0_DbKi - FA_vel) * LocalVar%Kp_Float ! Mod made by A. Wright: use the gain scheduled value of KPfloat in the floating fb equ's below (instead of the old value of CntrPar%Fl_Kp), for either value of CntrPar%Fl_Mode...
        ELSEIF (CntrPar%Fl_Mode == 2) THEN
            FloatingFeedback = (0.0_DbKi - NacIMU_FA_vel) *LocalVar%Kp_Float ! Mod made by A. Wright: use the gain scheduled value of KPfloat in the floating fb equ's below (instead of the old value of CntrPar%Fl_Kp), for either value of CntrPar%Fl_Mode...
        END IF

    END FUNCTION FloatingFeedback
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
        ! Yaw rate controller
        !       Y_ControlMode = 0, No yaw control
        !       Y_ControlMode = 1, Simple yaw rate control using yaw drive
        !       Y_ControlMode = 2, Yaw by IPC (accounted for in IPC subroutine)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
    
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*) ! The swap array, used to pass data to, and receive data from, the DLL controller.
    
        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        ! Internal Variables
        INTEGER(IntKi)              :: K
        REAL(DbKi)                  :: RootMyb_Vel(3)
        REAL(DbKi)                  :: RootMyb_VelErr(3)
        REAL(DbKi)                  :: axisTilt_1P, axisYaw_1P    ! Direct axis and quadrature axis outputted by Coleman transform, 1P
        REAL(DbKi)                  :: Flp_axisTilt_1P, Flp_axisYaw_1P ! Flap command in direct and quadrature axis coordinates
        ! Flap control
        IF (CntrPar%Flp_Mode > 0) THEN
            IF (LocalVar%iStatus == 0) THEN
                LocalVar%RootMyb_Last(1) = 0 - LocalVar%rootMOOP(1)
                LocalVar%RootMyb_Last(2) = 0 - LocalVar%rootMOOP(2)
                LocalVar%RootMyb_Last(3) = 0 - LocalVar%rootMOOP(3)
                ! Initial Flap angle
                LocalVar%Flp_Angle(1) = CntrPar%Flp_Angle
                LocalVar%Flp_Angle(2) = CntrPar%Flp_Angle
                LocalVar%Flp_Angle(3) = CntrPar%Flp_Angle
                ! Initialize controller
                IF (CntrPar%Flp_Mode == 2) THEN
                    LocalVar%Flp_Angle(K) = PIIController(RootMyb_VelErr(K), 0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, 0.05_DbKi, -CntrPar%Flp_MaxPit , CntrPar%Flp_MaxPit , LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
                ENDIF
            
            ! Steady flap angle
            ELSEIF (CntrPar%Flp_Mode == 1) THEN
                LocalVar%Flp_Angle(1) = LocalVar%Flp_Angle(1) 
                LocalVar%Flp_Angle(2) = LocalVar%Flp_Angle(2) 
                LocalVar%Flp_Angle(3) = LocalVar%Flp_Angle(3) 

            ! PII flap control
            ELSEIF (CntrPar%Flp_Mode == 2) THEN
                DO K = 1,LocalVar%NumBl
                    ! Find flap angle command - includes an integral term to encourage zero flap angle
                    LocalVar%Flp_Angle(K) = PIIController(-LocalVar%rootMOOPF(K), 0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, 0.05_DbKi, -CntrPar%Flp_MaxPit , CntrPar%Flp_MaxPit , LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
                    ! Saturation Limits
                    LocalVar%Flp_Angle(K) = saturate(LocalVar%Flp_Angle(K), -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit) * R2D
                END DO

            ! Cyclic flap Control
            ELSEIF (CntrPar%Flp_Mode == 3) THEN
                ! Pass rootMOOPs through the Coleman transform to get the tilt and yaw moment axis
                CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, NP_1, axisTilt_1P, axisYaw_1P)

                ! Apply PI control
                Flp_axisTilt_1P = PIController(axisTilt_1P, CntrPar%Flp_Kp, CntrPar%Flp_Ki, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
                Flp_axisYaw_1P = PIController(axisYaw_1P, CntrPar%Flp_Kp, CntrPar%Flp_Ki, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI) 
            
                ! Pass direct and quadrature axis through the inverse Coleman transform to get the commanded pitch angles
                CALL ColemanTransformInverse(Flp_axisTilt_1P, Flp_axisYaw_1P, LocalVar%Azimuth, NP_1, 0.0_DbKi, LocalVar%Flp_Angle)
                
            ENDIF

            ! Send to AVRSwap
            avrSWAP(120) = LocalVar%Flp_Angle(1)   ! Send flap pitch command (deg)
            avrSWAP(121) = LocalVar%Flp_Angle(2)   ! Send flap pitch command (deg)
            avrSWAP(122) = LocalVar%Flp_Angle(3)   ! Send flap pitch command (deg)
        ELSE
            RETURN
        ENDIF
    END SUBROUTINE FlapControl


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
        ! Active wake controller
        !       AWC_Mode = 0, No active wake control
        !       AWC_Mode = 1, SNL active wake control
        !       AWC_Mode = 2, Coleman Transform-based active wake control
        !       AWC_Mode = 3, Closed-loop Proportional-integral (PI) active wake control
        !       AWC_Mode = 4, Closed-loop Proportional-resonant (PR) active wake control
        !       AWC_Mode = 5, Strouhal transformation based closed-loop active wake control
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances

        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(DebugVariables), INTENT(INOUT)       :: DebugVar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst

        ! Local vars
        REAL(DbKi), PARAMETER      :: phi1 = 0.0                       ! Phase difference from first to first blade
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade
        REAL(DbKi), DIMENSION(3)      :: AWC_angle
        COMPLEX(DbKi), DIMENSION(3)   :: AWC_complexangle
        COMPLEX(DbKi)              :: complexI = (0.0, 1.0)
        INTEGER(IntKi)             :: Imode, K                         ! Index used for looping through AWC modes, blades
        REAL(DbKi)                 :: clockang                         ! Clock angle for AWC pitching
        REAL(DbKi)                 :: omega                            ! angular frequency for AWC pitching in Hz
        REAL(DbKi)                 :: amp                              ! amplitude for AWC pitching in degrees
        REAL(DbKi), DIMENSION(2)   :: AWC_TiltYaw = [0.0, 0.0]         ! AWC tilt and yaw pitch signals
        REAL(DbKi), DIMENSION(2)   :: Error = [0.0, 0.0]               ! Error in transformed tilt and yaw signals
        REAL(DbKi), DIMENSION(2)   :: FixedFrameM                      ! Measured tilt moment
        REAL(DbKi)                 :: StrAzimuth                       ! Strouhal transformed "azimuth" angle
        REAL(DbKi)                 :: StartTime = 0.0                  ! Start time of closed-loop AWC


        ! Compute the AWC pitch settings, complex number approach
        IF (CntrPar%AWC_Mode == 1) THEN

            LocalVar%AWC_complexangle = 0.0D0

            DO Imode = 1,CntrPar%AWC_NumModes
                clockang = CntrPar%AWC_clockangle(Imode)*PI/180.0_DbKi
                omega = CntrPar%AWC_freq(Imode)*PI*2.0_DbKi
                AWC_angle(1) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi1 + clockang)
                AWC_angle(2) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi2 + clockang)
                AWC_angle(3) = omega * LocalVar%Time - CntrPar%AWC_n(Imode) * (LocalVar%Azimuth + phi3 + clockang)
                ! Add the forcing contribution to LocalVar%AWC_complexangle
                amp = CntrPar%AWC_amp(Imode)*PI/180.0_DbKi
                DO K = 1,LocalVar%NumBl ! Loop through all blades
                    LocalVar%AWC_complexangle(K) = LocalVar%AWC_complexangle(K) + amp * EXP(complexI * (AWC_angle(K)))
                END DO
            END DO

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + REAL(LocalVar%AWC_complexangle(K))
            END DO

        ! Open-loop Coleman transform method
        ELSEIF (CntrPar%AWC_Mode == 2) THEN

            ! Calculate reference OL blade signals
            AWC_TiltYaw = [0.0, 0.0]
            DO Imode = 1,CntrPar%AWC_NumModes
                AWC_TiltYaw(Imode) = D2R*CntrPar%AWC_amp(Imode)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(Imode) + CntrPar%AWC_clockangle(Imode)*D2R)
                IF (CntrPar%AWC_NumModes == 1) THEN
                    AWC_TiltYaw(2) = D2R*CntrPar%AWC_amp(1)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(1) + 2*CntrPar%AWC_clockangle(1)*D2R)
                ENDIF
                
                ! Inverse Coleman Transformation with phase offset
                CALL ColemanTransformInverse(AWC_TiltYaw(1), AWC_TiltYaw(2), LocalVar%Azimuth, CntrPar%AWC_harmonic(Imode), CntrPar%AWC_phaseoffset*D2R, AWC_angle)
            END DO

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
            END DO

            ! DEBUG VARIABLES
            DebugVar%axisTilt_2P = AWC_TiltYaw(1)
            DebugVar%axisYaw_2P = AWC_TiltYaw(2)
            CALL ColemanTransform(LocalVar%BlPitch, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), AWC_TiltYaw(1), AWC_TiltYaw(2))

            DebugVar%axisTilt_1P = AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = AWC_TiltYaw(2)
        
        ! Closed-loop PI / PR controller
        ELSEIF ((CntrPar%AWC_Mode == 3) .OR. (CntrPar%AWC_Mode == 4)) THEN

            !! If AWC_harmonic > 0, use AWC_NumModes=2
        
            CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), FixedFrameM(1), FixedFrameM(2))

            IF (CntrPar%AWC_harmonic(1) == 0) THEN
                ! Calculate mean moments, subtract later to get zero-mean tilt and yaw
                LocalVar%TiltMean = LocalVar%TiltMean + FixedFrameM(1)
                StartTime = 1/CntrPar%AWC_freq(1)
            ENDIF

            IF (LocalVar%Time .GT. StartTime) THEN
                DO Imode = 1,CntrPar%AWC_NumModes
                    Error(Imode) = CntrPar%AWC_amp(Imode)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(Imode) + CntrPar%AWC_clockangle(Imode)*D2R) &
                                        + (FixedFrameM(Imode) - LocalVar%TiltMean/(LocalVar%n_DT+1))

                    IF (CntrPar%AWC_Mode == 4) THEN
                        AWC_TiltYaw(Imode) = ResController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%AWC_freq(Imode), & 
                                                            CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, LocalVar%resP, LocalVar%restart, objInst%instRes)
                    ELSE
                        AWC_TiltYaw(Imode) = PIController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%PC_MinPit, CntrPar%PC_MaxPit, &
                                                            LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
                    ENDIF
                ENDDO
            ENDIF

            ! Pass tilt and yaw axis through the inverse Coleman transform to get the commanded pitch angles
            CALL ColemanTransformInverse(AWC_TiltYaw(1), AWC_TiltYaw(2), &
                                         LocalVar%Azimuth, CntrPar%AWC_harmonic(1), CntrPar%AWC_phaseoffset*D2R, AWC_angle)
            
            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                IF (CntrPar%AWC_harmonic(1) == 0) THEN
                    LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_TiltYaw(1)
                ELSE
                    LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
                ENDIF
            END DO

            ! DEBUG VARIABLES
            DebugVar%axisTilt_1P = AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = -FixedFrameM(1) + LocalVar%TiltMean/(LocalVar%n_DT+1)
            DebugVar%axisTilt_2P = CntrPar%AWC_amp(1)*sin(LocalVar%Time*2*PI*CntrPar%AWC_freq(1) + CntrPar%AWC_clockangle(1)*D2R)
            DebugVar%axisYaw_2P = Error(1)

        ! Closed-loop Strouhal transform method
        ELSEIF (CntrPar%AWC_Mode == 5) THEN

            StrAzimuth = wrap_360(360*LocalVar%Time*CntrPar%AWC_freq(1))*D2R

            CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, CntrPar%AWC_harmonic(1), FixedFrameM(1), FixedFrameM(2))
            
            ! Calculate mean tilt and yaw moments to subtract
            LocalVar%TiltMean = LocalVar%TiltMean + FixedFrameM(1)
            LocalVar%YawMean = LocalVar%YawMean + FixedFrameM(2)

            ! Calculate error with zero-mean moments
            Error(1) = CntrPar%AWC_amp(1) + sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*(FixedFrameM(1) - LocalVar%TiltMean/(LocalVar%n_DT+1)) &
                        + sin(StrAzimuth + CntrPar%AWC_clockangle(2)*D2R)*(FixedFrameM(2) - LocalVar%YawMean/(LocalVar%n_DT+1)) 

            ! PI Control
            IF (LocalVar%Time .GT. 1/CntrPar%AWC_freq(1)) THEN
                AWC_TiltYaw(1) = PIController(Error(1), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), &
                                    CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
            ENDIF

            ! Pass tilt and yaw axis through the inverse Strouhal + Coleman transform to get the commanded pitch angles
            CALL ColemanTransformInverse(sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*AWC_TiltYaw(1), & ! Tilt signal (inverse Str transform)
                                            sin(StrAzimuth + CntrPar%AWC_clockangle(2)*D2R)*AWC_TiltYaw(1), & ! Yaw signal (inverse Str transform)
                                            LocalVar%Azimuth, CntrPar%AWC_harmonic(1), CntrPar%AWC_phaseoffset*D2R, AWC_angle)

            DO K = 1,LocalVar%NumBl ! Loop through all blades, apply AWC_angle
                LocalVar%PitCom(K) = LocalVar%PitCom(K) + AWC_angle(K)
            END DO

            ! DEBUG VARIABLES
            DebugVar%axisTilt_1P = sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)*AWC_TiltYaw(1)
            DebugVar%axisYaw_1P = -FixedFrameM(1) + LocalVar%TiltMean/(LocalVar%n_DT+1)
            DebugVar%axisTilt_2P = CntrPar%AWC_amp(1)*sin(StrAzimuth + CntrPar%AWC_clockangle(1)*D2R)
            DebugVar%axisYaw_2P = Error(1)


        ENDIF

    END SUBROUTINE ActiveWakeControl

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        ! Cable controller
        !       CC_Mode = 0, No cable control, this code not executed
        !       CC_Mode = 1, User-defined cable control
        !       CC_Mode = 2, Position control, not yet implemented
        !
        ! Note that LocalVar%CC_Actuated*(), and CC_Desired() has a fixed max size of 12, which can be increased in rosco_types.yaml
        !
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
    
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*) ! The swap array, used to pass data to, and receive data from, the DLL controller.
    
        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        TYPE(ErrorVariables), INTENT(INOUT)      :: ErrVar
        
        ! Internal Variables
        Integer(IntKi)                            :: I_GROUP
        CHARACTER(*),               PARAMETER           :: RoutineName = 'StructuralControl'



        IF (CntrPar%CC_Mode == 1) THEN
            ! User defined control

            IF (LocalVar%Time > 500) THEN
                ! Shorten first group by 4 m
                LocalVar%CC_DesiredL(1) = -14.51
                LocalVar%CC_DesiredL(2) = 1.58
                LocalVar%CC_DesiredL(3) = -10.332


            END IF

        ELSEIF (CntrPar%CC_Mode == 2) THEN
            ! Open loop control
            ! DO I_GROUP = 1, CntrPar%CC_Group_N
            !     LocalVar%CC_DesiredL(I_GROUP) = interp1d(x,y,eq,ErrVar)

            DO I_GROUP = 1,CntrPar%CC_Group_N
                IF (CntrPar%Ind_CableControl(I_GROUP) > 0) THEN
                    LocalVar%CC_DesiredL(I_GROUP) = interp1d(CntrPar%OL_Breakpoints, &
                                                            CntrPar%OL_CableControl(I_GROUP,:), &
                                                            LocalVar%Time,ErrVar)
                ENDIF
            ENDDO



        END IF

        ! Convert desired to actuated line length and delta length for all groups

        DO I_GROUP = 1, CntrPar%CC_Group_N

            ! Get Actuated deltaL
            LocalVar%CC_ActuatedDL(I_GROUP) = SecLPFilter_Vel(LocalVar%CC_DesiredL(I_GROUP),LocalVar%DT,2*PI/CntrPar%CC_ActTau,REAL(1.0,DbKi), &
                                                                LocalVar%FP,LocalVar%iStatus,LocalVar%restart,objInst%instSecLPFV)

            ! Integrate
            LocalVar%CC_ActuatedL(I_GROUP) = PIController(LocalVar%CC_ActuatedDL(I_GROUP),0.0_DbKi,1.0_DbKi, &
                                                    -1000.0_DbKi,1000.0_DbKi,LocalVar%DT,LocalVar%CC_ActuatedDL(1), &
                                                    LocalVar%piP, LocalVar%restart, objInst%instPI)

        END DO

        ! Assign to avrSWAP
        DO I_GROUP = 1, CntrPar%CC_Group_N

            avrSWAP(CntrPar%CC_GroupIndex(I_GROUP)) = LocalVar%CC_ActuatedL(I_GROUP)
            avrSWAP(CntrPar%CC_GroupIndex(I_GROUP)+1) = LocalVar%CC_ActuatedDL(I_GROUP)

        END DO

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE CableControl

!-------------------------------------------------------------------------------------------------------------------------------
SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        ! Cable controller
        !       StC_Mode = 0, No cable control, this code not executed
        !       StC_Mode = 1, User-defined cable control
        !       StC_Mode = 2, Ballast-like control, not yet implemented
        !
        ! Note that LocalVar%StC_Input() has a fixed max size of 12, which can be increased in rosco_types.yaml
        !
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
    
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*) ! The swap array, used to pass data to, and receive data from, the DLL controller.
    
        TYPE(ControlParameters), INTENT(INOUT)    :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)       :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT)      :: objInst
        TYPE(ErrorVariables), INTENT(INOUT)      :: ErrVar

        
        ! Internal Variables
        Integer(IntKi)                            :: I_GROUP
        CHARACTER(*),               PARAMETER           :: RoutineName = 'StructuralControl'



        IF (CntrPar%StC_Mode == 1) THEN
            ! User defined control, step example

            IF (LocalVar%Time > 500) THEN
                ! Step change in input of -4500 N
                LocalVar%StC_Input(1) = -1.234e+06
                LocalVar%StC_Input(2) = 2.053e+06
                LocalVar%StC_Input(3) = -7.795e+05

            END IF


        ELSEIF (CntrPar%StC_Mode == 2) THEN


            DO I_GROUP = 1,CntrPar%StC_Group_N
                IF (CntrPar%Ind_StructControl(I_GROUP) > 0) THEN
                    LocalVar%StC_Input(I_GROUP) =  interp1d(CntrPar%OL_Breakpoints, &
                                                            CntrPar%OL_StructControl(I_GROUP,:), &
                                                            LocalVar%Time,ErrVar)
                ENDIF
            ENDDO



        END IF


        ! Assign to avrSWAP
        DO I_GROUP = 1, CntrPar%StC_Group_N
            avrSWAP(CntrPar%StC_GroupIndex(I_GROUP)) = LocalVar%StC_Input(I_GROUP)
        END DO

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE StructuralControl
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
        USE ROSCO_Types, ONLY : piParams

    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)      :: inst
        REAL(DbKi),    INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT)  :: piP
        LOGICAL,    INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            
            PIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIController


!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar)
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances

    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: kd
        REAL(DbKi),    INTENT(IN)         :: tf
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        TYPE(ObjectInstances),      INTENT(INOUT)   :: objInst  ! all object instances (PI, filters used here)
        TYPE(LocalVariables),       INTENT(INOUT)   :: LocalVar

        REAL(DbKi),    INTENT(IN)           :: I0
        TYPE(piParams), INTENT(INOUT)       :: piP
        LOGICAL,    INTENT(IN)              :: reset     
        
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                          :: PTerm, DTerm                                 ! Proportional, deriv. terms
        REAL(DbKi)                          :: EFilt                    ! Filtered error for derivative

        ! Always filter error
        EFilt = LPFilter(error, DT, tf, LocalVar%FP, LocalVar%iStatus, reset, objInst%instLPF)

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(objInst%instPI) = I0
            piP%ITermLast(objInst%instPI) = I0
            piP%ELast(objInst%instPI) = 0.0_DbKi
            PIDController = I0
        ELSE
            ! Proportional
            PTerm = kp*error
            
            ! Integrate and saturate
            piP%ITerm(objInst%instPI) = piP%ITerm(objInst%instPI) + DT*ki*error
            piP%ITerm(objInst%instPI) = saturate(piP%ITerm(objInst%instPI), minValue, maxValue)

            ! Derivative (filtered)
            DTerm = kd * (EFilt - piP%ELast(objInst%instPI)) / DT
            
            ! Saturate all
            PIDController = saturate(PTerm + piP%ITerm(objInst%instPI) + DTerm, minValue, maxValue)
        
            ! Save lasts
            piP%ITermLast(objInst%instPI) = piP%ITerm(objInst%instPI)
            piP%ELast(objInst%instPI) = EFilt
        END IF
        objInst%instPI = objInst%instPI + 1
        
    END FUNCTION PIDController

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst)
    ! PI controller, with output saturation. 
    ! Added error2 term for additional integral control input
        USE ROSCO_Types, ONLY : piParams
        
        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi), INTENT(IN)         :: error
        REAL(DbKi), INTENT(IN)         :: error2
        REAL(DbKi), INTENT(IN)         :: kp
        REAL(DbKi), INTENT(IN)         :: ki2
        REAL(DbKi), INTENT(IN)         :: ki
        REAL(DbKi), INTENT(IN)         :: minValue
        REAL(DbKi), INTENT(IN)         :: maxValue
        REAL(DbKi), INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)   :: inst
        REAL(DbKi), INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT) :: piP
        LOGICAL, INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term

        ! Initialize persistent variables/arrays, and set inital condition for integrator term
        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            piP%ITerm2(inst) = I0
            piP%ITermLast2(inst) = I0
            
            PIIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm2(inst) = piP%ITerm2(inst) + DT*ki2*error2
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            piP%ITerm2(inst) = saturate(piP%ITerm2(inst), minValue, maxValue)
            PIIController = PTerm + piP%ITerm(inst) + piP%ITerm2(inst)
            PIIController = saturate(PIIController, minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIIController

        !-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst)
        USE ROSCO_Types, ONLY : resParams

    ! PI controller, with output saturation

        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: freq
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)     :: inst
        TYPE(resParams), INTENT(INOUT)    :: resP
        LOGICAL,    INTENT(IN)            :: reset
        ! Allocate local variables
        REAL(DbKi)                        :: omega                                        ! Frequency
        REAL(DbKi)                        :: a0, a1, a2, b0, b1, b2

        omega = 2*PI*freq

        !! Tustin RC
        b0 = 4+omega**2*DT**2
        b1 = -8+2*omega**2*DT**2
        b2 = 4+omega**2*DT**2
        a0 = b0*kp + 2*DT*ki
        a1 = b1*kp 
        a2 = b2*kp - 2*DT*ki
        ! Initialize persistent variables/arrays, and set initial condition for integrator term
        IF (reset) THEN
            resP%res_OutputSignalLast1(inst)  = 0
            resP%res_OutputSignalLast2(inst)  = 0
            resP%res_InputSignalLast1(inst)   = 0
            resP%res_InputSignalLast2(inst)   = 0
        ELSE
            ResController = 1/b0*( -b1*resP%res_OutputSignalLast1(inst) - b2*resP%res_OutputSignalLast2(inst) &
                                    + a0*error + a1*resP%res_InputSignalLast1(inst) + a2*resP%res_InputSignalLast2(inst))
            ResController = saturate(ResController, minValue, maxValue)
        
            ! Save signals for next time step
            resP%res_InputSignalLast2(inst)   = resP%res_InputSignalLast1(inst)
            resP%res_InputSignalLast1(inst)   = error
            resP%res_OutputSignalLast2(inst)  = resP%res_OutputSignalLast1(inst)
            resP%res_OutputSignalLast1(inst)  = ResController
        END IF
        inst = inst + 1
        
    END FUNCTION ResController

!-------------------------------------------------------------------------------------------------------------------------------
END MODULE Controllers
