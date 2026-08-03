! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Read and set the parameters used by the controller

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

    USE Constants
    USE Functions
    USE Filters
    USE SysSubs
    USE ROSCO_Helpers
    IMPLICIT NONE


CONTAINS
 ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)
        USE ROSCO_Types, ONLY : LocalVariables

        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        TYPE(ControlParameters), INTENT(IN) :: CntrPar
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar

        ! Allocate Variables:
        INTEGER(IntKi)                                  :: K         ! Index used for looping through blades.

        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LocalVar%iStatus            = NINT(avrSWAP(1))
        LocalVar%Time               = avrSWAP(2)
        LocalVar%DT                 = avrSWAP(3)
        LocalVar%VS_MechGenPwr      = avrSWAP(14)
        LocalVar%VS_GenPwr          = avrSWAP(15)
        LocalVar%GenSpeed           = avrSWAP(20)
        LocalVar%RotSpeed           = avrSWAP(21)
        LocalVar%GenTqMeas          = avrSWAP(23)
        LocalVar%NacVane            = avrSWAP(24) * R2D
        LocalVar%HorWindV           = avrSWAP(27)
        LocalVar%rootMOOP(1)        = avrSWAP(30)
        LocalVar%rootMOOP(2)        = avrSWAP(31)
        LocalVar%rootMOOP(3)        = avrSWAP(32)
        LocalVar%NacHeading         = avrSWAP(37) * R2D
        LocalVar%FA_Acc_TT          = avrSWAP(53)  ! This is the translational acceleration of the tower top in the non-rotating frame
        LocalVar%SS_Acc_TT          = avrSWAP(54)  ! This is the translational acceleration of the tower top in the non-rotating frame
        LocalVar%NacIMU_FA_RAcc      = avrSWAP(83)  ! This is the rotational aceleration of the nacelle in the shaft frame
        LocalVar%Azimuth            = avrSWAP(60)
        LocalVar%NumBl              = NINT(avrSWAP(61))

        if (CntrPar%Ext_Interface > 0) THEN ! Use extended bladed interface

            ! Platform signals 
            LocalVar%PtfmTDX            = avrSWAP(1001)
            LocalVar%PtfmTDY            = avrSWAP(1002)
            LocalVar%PtfmTDZ            = avrSWAP(1003)
            LocalVar%PtfmRDX            = avrSWAP(1004)
            LocalVar%PtfmRDY            = avrSWAP(1005)
            LocalVar%PtfmRDZ            = avrSWAP(1006)
            LocalVar%PtfmTVX            = avrSWAP(1007)
            LocalVar%PtfmTVY            = avrSWAP(1008)
            LocalVar%PtfmTVZ            = avrSWAP(1009)
            LocalVar%PtfmRVX            = avrSWAP(1010)
            LocalVar%PtfmRVY            = avrSWAP(1011)
            LocalVar%PtfmRVZ            = avrSWAP(1012)
            LocalVar%PtfmTAX            = avrSWAP(1013)
            LocalVar%PtfmTAY            = avrSWAP(1014)
            LocalVar%PtfmTAZ            = avrSWAP(1015)
            LocalVar%PtfmRAX            = avrSWAP(1016)
            LocalVar%PtfmRAY            = avrSWAP(1017)
            LocalVar%PtfmRAZ            = avrSWAP(1018)

        ENDIF


        ! Check that we haven't already loaded this dynamic library
        IF (LocalVar%iStatus == 0) THEN
            IF (LocalVar%AlreadyInitialized == 0) THEN
                LocalVar%AlreadyInitialized = 1
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'ERROR: This ROSCO dynamic library has already been loaded.'
                RETURN
            ENDIF
        ENDIF


        

        ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%BlPitch(1) = avrSWAP(4)
            LocalVar%BlPitch(2) = avrSWAP(33)
            LocalVar%BlPitch(3) = avrSWAP(34)
        ELSE
            
            ! Subtract pitch actuator fault for blade K - This in a sense would make the controller blind to the pitch fault
            IF (CntrPar%PF_Mode == 1) THEN
                DO K = 1, LocalVar%NumBl
                    ! This assumes that the pitch actuator fault is hardware fault
                    LocalVar%BlPitch(K) = LocalVar%PitComAct(K) - CntrPar%PF_Offsets(K) ! why is PitCom used and not PitComAct??
                END DO
            ELSE
                LocalVar%BlPitch(1) = LocalVar%PitComAct(1)
                LocalVar%BlPitch(2) = LocalVar%PitComAct(2)
                LocalVar%BlPitch(3) = LocalVar%PitComAct(3)     
            END IF

        ENDIF

        LocalVar%BlPitchCMeas = (1 / REAL(LocalVar%NumBl)) * (LocalVar%BlPitch(1) + LocalVar%BlPitch(2) + LocalVar%BlPitch(3)) 

        IF (LocalVar%iStatus == 0) THEN     ! TODO: Technically, LocalVar%Time > 0, too, but this restart is in many places as a reset
            LocalVar%restart = .True.
        ELSE
            LocalVar%restart = .False.
        ENDIF

        ! FA_Acc_TT is in the non-rotating tower-top frame, so we need to convert it to the rotating (nacelle) frame of reference
        LocalVar%FA_Acc_Nac = LocalVar%FA_Acc_TT * COS(LocalVar%NacHeading * D2R) + LocalVar%SS_Acc_TT * SIN(LocalVar%NacHeading * D2R)


        ! Increment timestep counter
        IF (LocalVar%iStatus == 0 .AND. LocalVar%Time == 0) THEN
            LocalVar%n_DT = 0
        ELSE
            LocalVar%n_DT = LocalVar%n_DT + 1
        ENDIF

    END SUBROUTINE ReadAvrSWAP    
! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar)
                
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData, ErrorVariables
        
        REAL(ReKi),                 INTENT(INOUT)   :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(C_CHAR),          INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file

        INTEGER(IntKi),             INTENT(IN   )   :: size_avcMSG
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)   :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)   :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar
        CHARACTER(NINT(avrSWAP(50))-1), INTENT(IN)  :: RootName 

        
        INTEGER(IntKi)                              :: K, I, I_OL    ! Index used for looping through blades.
        CHARACTER(1024)                             :: OL_String                    ! Open description loop string
        INTEGER(IntKi)                              :: OL_Count                     ! Number of open loop channels
        INTEGER(IntKi)                              :: UnOpenLoop       ! Open Loop file unit
        INTEGER(IntKi)                              :: N_OL_Cables
        INTEGER(IntKi)                              :: N_OL_StCs

        CHARACTER(*),               PARAMETER       :: RoutineName = 'SetParameters'

        
        
        ! Error Catching Variables
        ! Set ErrVar%aviFAIL to 0 in each iteration:
        ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        ErrVar%size_avcMSG  = size_avcMSG
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF         = 1
        objInst%instSecLPF      = 1
        objInst%instSecLPFV     = 1
        objInst%instHPF         = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch       = 1
        objInst%instPI          = 1
        objInst%instRes         = 1
        objInst%instRL          = 1
        
        ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
        avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
        avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
        avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
        avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
        avrSWAP(55) = 0.0 ! Pitch override: 0=yes
        avrSWAP(56) = 0.0 ! Torque override: 0=yes
        avrSWAP(65) = 0.0 ! Number of variables returned for logging
        avrSWAP(72) = 0.0 ! Generator start-up resistance
        avrSWAP(79) = 4.0 ! Request for loads: 0=none
        avrSWAP(80) = 0.0 ! Variable slip current status
        avrSWAP(81) = 0.0 ! Variable slip current demand
        
        ! Read any External Controller Parameters specified in the User Interface
        !   and initialize variables:
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Inform users that we are using this user-defined routine:
            ! ErrVar%aviFAIL = 1
            write (*,*) '                                                                              '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                        'Running ROSCO-'//TRIM(rosco_version)//NEW_LINE('A')// &
                        'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                        'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                        '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'
            ! Specifically save accINFILE info (DISCON.IN)
            LocalVar%ACC_INFILE_SIZE = NINT(avrSWAP(50))
            Allocate(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
            LocalVar%ACC_INFILE = accINFILE

            ! Read Control Parameter File
            CALL ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, NINT(avrSWAP(50)), RootName, ErrVar)
            ! If there's been an file reading error, don't continue
            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
                RETURN
            ENDIF

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData,ErrVar)
            ENDIF
        
            ! Initialize the Local variables:
            LocalVar%PitCom     = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call

            ! Wind speed estimator initialization
            LocalVar%WE_Vw      = LocalVar%HorWindV
            LocalVar%WE_VwI     = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            LocalVar%WE_Op      = 1
            LocalVar%WE_Op_Last = 1
            
            ! Setpoint Smoother initialization to zero
            LocalVar%SS_DelOmegaF = 0

            IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
                ! Generator Torque at K omega^2 or rated
                IF (LocalVar%GenSpeed > 0.98 * CntrPar%PC_RefSpd) THEN
                    LocalVar%GenTq = CntrPar%VS_RtTq
                ELSE
                    LocalVar%GenTq = min(CntrPar%VS_RtTq, CntrPar%VS_Rgn2K*LocalVar%GenSpeed*LocalVar%GenSpeed)
                ENDIF
            ELSE
                ! Set torque initial condition based on operating schedule at current wind speed
                LocalVar%GenTq = interp1d(CntrPar%VS_FBP_U, CntrPar%VS_FBP_Tau, LocalVar%HorWindV, ErrVar)
            ENDIF
            LocalVar%VS_LastGenTrq = LocalVar%GenTq
            LocalVar%VS_MaxTq      = CntrPar%VS_MaxTq
            LocalVar%VS_GenPwr     = LocalVar%GenTq * LocalVar%GenSpeed * CntrPar%VS_GenEff/100.0
            
            ! Initialize variables
            LocalVar%CC_DesiredL = 0
            LocalVar%CC_ActuatedL = 0
            LocalVar%CC_ActuatedDL = 0
            LocalVar%StC_Input = 0

            LocalVar%ZMQ_YawOffset = 0
            LocalVar%ZMQ_PitOffset = 0
            LocalVar%ZMQ_ID = CntrPar%ZMQ_ID

            ! Check validity of input parameters:
            CALL CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)

            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            ENDIF


        ENDIF

        ! Initialize other values on each timestep
        
        ! Open Loop index
        IF (CntrPar%OL_BP_Mode == 0) THEN
            LocalVar%OL_Index = LocalVar%Time

        ELSE
            ! Wind speed, OL_BP_Mode = 1
            LocalVar%OL_Index = LocalVar%WE_Vw
            IF (CntrPar%OL_BP_FiltFreq > 0) THEN
                LocalVar%OL_Index = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%OL_BP_FiltFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            ENDIF

        ENDIF

    END SUBROUTINE SetParameters
    
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    ! Also, all computed CntrPar%* parameters should be computed in this subroutine
    SUBROUTINE ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters, ErrorVariables, LocalVariables

        INTEGER(IntKi)                                  :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)    ! DISCON input filename
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar                      ! Control parameter type
        TYPE(LocalVariables),        INTENT(INOUT)       :: LocalVar                       ! Control parameter type
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar                       ! Control parameter type
        CHARACTER(accINFILE_size),  INTENT(IN)          :: RootName 

        
        INTEGER(IntKi)                                  :: UnControllerParameters  ! Unit number to open file

        CHARACTER(1024)                                 :: PriPath        ! Path name of the primary DISCON file

        CHARACTER(2048)                                 :: TmpLine        ! Path name of the primary DISCON file
        INTEGER(IntKi)                                  :: NumLines, IOS, I_LINE, ErrStat
        INTEGER(IntKi)                                  :: UnEc
        CHARACTER(128)                                  :: EchoFilename              ! Input checkpoint file
        CHARACTER(MaxLineLength), DIMENSION(:), ALLOCATABLE      :: FileLines

        INTEGER(IntKi)                                  :: I, I_OL    ! Index used for looping through blades.
        CHARACTER(1024)                                 :: OL_String                    ! Open description loop string
        INTEGER(IntKi)                                  :: OL_Count                     ! Number of open loop channels
        INTEGER(IntKi)                                  :: UnOpenLoop       ! Open Loop file unit
        INTEGER(IntKi)                                  :: N_OL_Cables
        INTEGER(IntKi)                                  :: N_OL_StCs
        
        CHARACTER(*),               PARAMETER           :: RoutineName = 'ReadControlParameterFileSub'

        ! Get primary path of DISCON.IN file (accINFILE(1) here)
        CALL GetPath( accINFILE(1), PriPath )     ! Input files will be relative to the path where the primary input file is located.
        CALL GetNewUnit(UnControllerParameters, ErrVar)
        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')

        ! Read all lines, first get the number of lines
        NumLines = 0
        IOS = 0
        DO WHILE (IOS == 0)  ! read the rest of the file (until an error occurs)
            NumLines = NumLines + 1
            READ(UnControllerParameters,'(A)',IOSTAT=IOS) TmpLine        
        END DO !WHILE

        ALLOCATE(FileLines(NumLines))
        REWIND( UnControllerParameters )

        DO I_LINE = 1,NumLines
            READ(UnControllerParameters,'(A)',IOSTAT=IOS) FileLines(I_LINE)
        END DO

        ! Close Input File
        CLOSE(UnControllerParameters)

        ! Read Echo first, so file can be set up, if desired
        CALL ParseInput(FileLines,'Echo',           CntrPar%Echo,               accINFILE(1), ErrVar)
        IF (ErrVar%aviFAIL < 0) RETURN

        ! Set up echo file
        UnEc = 0
        IF (CntrPar%Echo > 0) THEN
            EchoFilename = TRIM(RootName)//'.RO.echo'
            CALL GetNewUnit(UnEc, ErrVar)
            OPEN(unit=UnEc, FILE=TRIM(EchoFilename), IOSTAT=ErrStat, ACTION='WRITE' )
            IF ( ErrStat /= 0 ) THEN
                ErrVar%ErrMsg  = 'Cannot open file '//TRIM( EchoFilename )//'. Another program may have locked it for writing.'
                ErrVar%aviFAIL = 1
            ELSE
                WRITE( UnEc, *) 'ROSCO ECHO file'
                WRITE( UnEc, *) 'Generated on '//CurDate()//' at '//CurTime()//' using ROSCO-'//TRIM(rosco_version)
                WRITE( UnEc, *)  NEW_LINE('A')
                WRITE( UnEc, *) 'Line Number',Tab,'Parameter',Tab,'Value'
                WRITE( UnEc, *) '-----------------------------------------'
            ENDIF
        ENDIF

        !----------------------- Simulation Control --------------------------
        CALL ParseInput(FileLines,'LoggingLevel',   CntrPar%LoggingLevel,       accINFILE(1), ErrVar, .TRUE., UnEc=UnEc)
        CALL ParseInput(FileLines,'DT_Out',         CntrPar%DT_Out,             accINFILE(1), ErrVar, .TRUE., UnEc=UnEc)
        CALL ParseInput(FileLines,'Ext_Interface',  CntrPar%Ext_Interface,      accINFILE(1), ErrVar, .TRUE., UnEc=UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !----------------- CONTROLLER FLAGS ---------------------
        CALL ParseInput(FileLines,'F_LPFType',       CntrPar%F_LPFType,         accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'IPC_ControlMode', CntrPar%IPC_ControlMode,   accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'VS_ControlMode',  CntrPar%VS_ControlMode,    accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'VS_ConstPower',   CntrPar%VS_ConstPower,     accINFILE(1), ErrVar, .TRUE., UnEc=UnEc) ! Default is 0
        CALL ParseInput(FileLines,'VS_FBP',          CntrPar%VS_FBP,            accINFILE(1), ErrVar, .TRUE., UnEc=UnEc) ! Default is 0
        CALL ParseInput(FileLines,'PC_ControlMode',  CntrPar%PC_ControlMode,    accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'Y_ControlMode',   CntrPar%Y_ControlMode,     accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'SS_Mode',         CntrPar%SS_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'PRC_Mode',        CntrPar%PRC_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'WE_Mode',         CntrPar%WE_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'PS_Mode',         CntrPar%PS_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'SU_Mode',         CntrPar%SU_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'SD_Mode',         CntrPar%SD_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'FL_Mode',         CntrPar%FL_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'TD_Mode',         CntrPar%TD_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'TRA_Mode',        CntrPar%TRA_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'Flp_Mode',        CntrPar%Flp_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'OL_Mode',         CntrPar%OL_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'PA_Mode',         CntrPar%PA_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'PF_Mode',         CntrPar%PF_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'AWC_Mode',        CntrPar%AWC_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
        CALL ParseInput(FileLines,'Ext_Mode',        CntrPar%Ext_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
		CALL ParseInput(FileLines,'ZMQ_Mode',        CntrPar%ZMQ_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
		CALL ParseInput(FileLines,'CC_Mode',         CntrPar%CC_Mode,           accINFILE(1), ErrVar, UnEc=UnEc)
		CALL ParseInput(FileLines,'StC_Mode',        CntrPar%StC_Mode,          accINFILE(1), ErrVar, UnEc=UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !----------------- FILTER CONSTANTS ---------------------
        CALL ParseInput(FileLines,  'F_LPFCornerFreq',      CntrPar%F_LPFCornerFreq,                             accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'F_LPFDamping',         CntrPar%F_LPFDamping,                                accINFILE(1), ErrVar, CntrPar%F_LPFType == 1, UnEc)
        CALL ParseInput(FileLines,  'F_NumNotchFilts',      CntrPar%F_NumNotchFilts,                             accINFILE(1), ErrVar, .TRUE., UnEc)
        CALL ParseAry(  FileLines,  'F_NotchFreqs',         CntrPar%F_NotchFreqs,       CntrPar%F_NumNotchFilts, accINFILE(1), ErrVar, CntrPar%F_NumNotchFilts == 0, UnEc)
        CALL ParseAry(  FileLines,  'F_NotchBetaNum',       CntrPar%F_NotchBetaNum,     CntrPar%F_NumNotchFilts, accINFILE(1), ErrVar, CntrPar%F_NumNotchFilts == 0, UnEc)
        CALL ParseAry(  FileLines,  'F_NotchBetaDen',       CntrPar%F_NotchBetaDen,     CntrPar%F_NumNotchFilts, accINFILE(1), ErrVar, CntrPar%F_NumNotchFilts == 0, UnEc)
        CALL ParseInput(FileLines,  'F_GenSpdNotch_N',      CntrPar%F_GenSpdNotch_N,                             accINFILE(1), ErrVar, CntrPar%F_NumNotchFilts == 0, UnEc)
        CALL ParseInput(FileLines,  'F_TwrTopNotch_N',      CntrPar%F_TwrTopNotch_N,                             accINFILE(1), ErrVar, CntrPar%F_NumNotchFilts == 0, UnEc)
        CALL ParseInput(FileLines,  'F_SSCornerFreq',       CntrPar%F_SSCornerFreq,                              accINFILE(1), ErrVar, CntrPar%SS_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'F_WECornerFreq',       CntrPar%F_WECornerFreq,                              accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'F_YawErr',             CntrPar%F_YawErr,                                    accINFILE(1), ErrVar, CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'F_FlCornerFreq',       CntrPar%F_FlCornerFreq,     2,                       accINFILE(1), ErrVar, CntrPar%FL_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'F_FlHighPassFreq',     CntrPar%F_FlHighPassFreq,                            accINFILE(1), ErrVar, CntrPar%FL_Mode == 0, UnEc)
        CALL ParseAry(  FileLines,  'F_FlpCornerFreq',      CntrPar%F_FlpCornerFreq,    2,                       accINFILE(1), ErrVar, CntrPar%Flp_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'F_VSRefSpdCornerFreq', CntrPar%F_VSRefSpdCornerFreq,                        accINFILE(1), ErrVar, CntrPar%VS_ControlMode < 2, UnEc)

        ! Optional filter inds
        IF (CntrPar%F_GenSpdNotch_N > 0) THEN
            CALL ParseAry(FileLines,    'F_GenSpdNotch_Ind',    CntrPar%F_GenSpdNotch_Ind,  CntrPar%F_GenSpdNotch_N, accINFILE(1), ErrVar, CntrPar%F_GenSpdNotch_N == 0, UnEc)
        ENDIF
        IF (CntrPar%F_TwrTopNotch_N > 0) THEN
            CALL ParseAry(FileLines,    'F_TwrTopNotch_Ind',    CntrPar%F_TwrTopNotch_Ind,  CntrPar%F_TwrTopNotch_N, accINFILE(1), ErrVar, CntrPar%F_TwrTopNotch_N == 0, UnEc)
        ENDIF

        IF (ErrVar%aviFAIL < 0) RETURN

        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------
        CALL ParseInput(FileLines,  'PC_GS_n',      CntrPar%PC_GS_n,                        accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PC_GS_angles', CntrPar%PC_GS_angles, CntrPar%PC_GS_n,  accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PC_GS_KP',     CntrPar%PC_GS_KP,     CntrPar%PC_GS_n,  accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PC_GS_KI',     CntrPar%PC_GS_KI,     CntrPar%PC_GS_n,  accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PC_GS_KD',     CntrPar%PC_GS_KD,     CntrPar%PC_GS_n,  accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PC_GS_TF',     CntrPar%PC_GS_TF,     CntrPar%PC_GS_n,  accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_MaxPit',    CntrPar%PC_MaxPit,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_MinPit',    CntrPar%PC_MinPit,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_MaxRat',    CntrPar%PC_MaxRat,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_MinRat',    CntrPar%PC_MinRat,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_RefSpd',    CntrPar%PC_RefSpd,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_FinePit',   CntrPar%PC_FinePit,                     accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'PC_Switch',    CntrPar%PC_Switch,                      accINFILE(1), ErrVar, CntrPar%PC_ControlMode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------------- IPC CONSTANTS -----------------------
        CALL ParseAry(  FileLines,  'IPC_Vramp',        CntrPar%IPC_Vramp,          2,  accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'IPC_SatMode',      CntrPar%IPC_SatMode,            accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'IPC_IntSat',       CntrPar%IPC_IntSat,             accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'IPC_KP',           CntrPar%IPC_KP,             2,  accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'IPC_KI',           CntrPar%IPC_KI,             2,  accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'IPC_aziOffset',    CntrPar%IPC_aziOffset,      2,  accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'IPC_CornerFreqAct',CntrPar%IPC_CornerFreqAct,      accINFILE(1),   ErrVar, CntrPar%IPC_ControlMode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ VS TORQUE CONTROL CONSTANTS ----------------
        CALL ParseInput(FileLines,  'VS_GenEff',    CntrPar%VS_GenEff,                  accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_ArSatTq',   CntrPar%VS_ArSatTq,                 accINFILE(1), ErrVar, CntrPar%VS_ControlMode .NE. 1, UnEc)
        CALL ParseInput(FileLines,  'VS_MaxRat',    CntrPar%VS_MaxRat,                  accINFILE(1), ErrVar, CntrPar%VS_ControlMode .NE. 1, UnEc)
        CALL ParseInput(FileLines,  'VS_MaxTq',     CntrPar%VS_MaxTq,                   accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_MinTq',     CntrPar%VS_MinTq,                   accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_MinOMSpd',  CntrPar%VS_MinOMSpd,                accINFILE(1), ErrVar)   ! Default 0 is fin, UnEce
        CALL ParseInput(FileLines,  'VS_Rgn2K',     CntrPar%VS_Rgn2K,                   accINFILE(1), ErrVar, CntrPar%VS_ControlMode == 2, UnEc)
        CALL ParseInput(FileLines,  'VS_RtPwr',     CntrPar%VS_RtPwr,                   accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_RtTq',      CntrPar%VS_RtTq,                    accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_RefSpd',    CntrPar%VS_RefSpd,                  accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_n',         CntrPar%VS_n,                       accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseAry(  FileLines,  'VS_KP',        CntrPar%VS_KP,      CntrPar%VS_n,   accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseAry(  FileLines,  'VS_KI',        CntrPar%VS_KI,      CntrPar%VS_n,   accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'VS_TSRopt',    CntrPar%VS_TSRopt,                  accINFILE(1), ErrVar, CntrPar%VS_ControlMode < 2, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ Fixed-Pitch Region 3 Control ------------
        CALL ParseInput(FileLines,  'VS_FBP_n',       CntrPar%VS_FBP_n,                          accINFILE(1), ErrVar, CntrPar%VS_FBP == VS_FBP_Variable_Pitch, UnEc)
        CALL ParseAry(  FileLines,  'VS_FBP_U',       CntrPar%VS_FBP_U,        CntrPar%VS_FBP_n, accINFILE(1), ErrVar, CntrPar%VS_FBP == VS_FBP_Variable_Pitch, UnEc)
        CALL ParseAry(  FileLines,  'VS_FBP_Omega',   CntrPar%VS_FBP_Omega,    CntrPar%VS_FBP_n, accINFILE(1), ErrVar, CntrPar%VS_FBP == VS_FBP_Variable_Pitch, UnEc)
        CALL ParseAry(  FileLines,  'VS_FBP_Tau',     CntrPar%VS_FBP_Tau,      CntrPar%VS_FBP_n, accINFILE(1), ErrVar, CntrPar%VS_FBP == VS_FBP_Variable_Pitch, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------- Setpoint Smoother --------------------------------
        CALL ParseInput(FileLines,  'SS_VSGain',    CntrPar%SS_VSGain,  accINFILE(1), ErrVar, CntrPar%SS_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SS_PCGain',    CntrPar%SS_PCGain,  accINFILE(1), ErrVar, CntrPar%SS_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ POWER REFERENCE TRACKING SETPOINTS --------------
        CALL ParseInput(FileLines,  'PRC_Comm',         CntrPar%PRC_Comm,                                   accINFILE(1), ErrVar, CntrPar%PRC_Mode .NE. 1, UnEc)
        CALL ParseInput(FileLines,  'PRC_R_Torque',     CntrPar%PRC_R_Torque,                               accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_Comm .NE. PRC_Comm_Constant), UnEc)
        CALL ParseInput(FileLines,  'PRC_R_Speed',      CntrPar%PRC_R_Speed,                                accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_Comm .NE. PRC_Comm_Constant), UnEc)
        CALL ParseInput(FileLines,  'PRC_R_Pitch',      CntrPar%PRC_R_Pitch,                                accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_Comm .NE. PRC_Comm_Constant), UnEc)
        CALL ParseInput(FileLines,  'PRC_Table_n',      CntrPar%PRC_Table_n,                                accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_R_Pitch == 1.0), UnEc)
        CALL ParseAry(FileLines,    'PRC_R_Table',      CntrPar%PRC_R_Table,        CntrPar%PRC_Table_n,    accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_R_Pitch == 1.0), UnEc)
        CALL ParseAry(FileLines,    'PRC_Pitch_Table',  CntrPar%PRC_Pitch_Table,    CntrPar%PRC_Table_n,    accINFILE(1), ErrVar, (CntrPar%PRC_Mode .NE. 2) .OR. (CntrPar%PRC_R_Pitch == 1.0), UnEc)
        CALL ParseInput(FileLines,  'PRC_n',            CntrPar%PRC_n,                            accINFILE(1), ErrVar,   CntrPar%PRC_Mode == 0)
        CALL ParseInput(FileLines,  'PRC_LPF_Freq',     CntrPar%PRC_LPF_Freq,                     accINFILE(1), ErrVar,   CntrPar%PRC_Mode == 0)
        CALL ParseAry(  FileLines,  'PRC_WindSpeeds',   CntrPar%PRC_WindSpeeds,   CntrPar%PRC_n,  accINFILE(1), ErrVar,   CntrPar%PRC_Mode == 0)
        CALL ParseAry(  FileLines,  'PRC_GenSpeeds',    CntrPar%PRC_GenSpeeds,    CntrPar%PRC_n,  accINFILE(1), ErrVar,   CntrPar%PRC_Mode == 0)

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------
        CALL ParseInput(FileLines,  'WE_BladeRadius',   CntrPar%WE_BladeRadius,                         accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'WE_Gamma',         CntrPar%WE_Gamma,                               accINFILE(1), ErrVar, CntrPar%WE_Mode .NE. 1, UnEc)
        CALL ParseInput(FileLines,  'WE_GearboxRatio',  CntrPar%WE_GearboxRatio,                        accINFILE(1), ErrVar, .FALSE., UnEc)
        CALL ParseInput(FileLines,  'WE_Jtot',          CntrPar%WE_Jtot,                                accINFILE(1), ErrVar, CntrPar%WE_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'WE_RhoAir',        CntrPar%WE_RhoAir,                              accINFILE(1), ErrVar, CntrPar%WE_Mode .NE. 2, UnEc)
        CALL ParseInput(FileLines,  'PerfFileName',     CntrPar%PerfFileName,                           accINFILE(1), ErrVar, CntrPar%WE_Mode == 0, UnEc )
        CALL ParseAry(  FileLines,  'PerfTableSize',    CntrPar%PerfTableSize,  2,                      accINFILE(1), ErrVar, CntrPar%WE_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'WE_FOPoles_N',     CntrPar%WE_FOPoles_N,                           accINFILE(1), ErrVar, CntrPar%WE_Mode .NE. 2, UnEc)
        CALL ParseAry(FileLines,    'WE_FOPoles_v',     CntrPar%WE_FOPoles_v,   CntrPar%WE_FOPoles_N,   accINFILE(1), ErrVar, CntrPar%WE_Mode .NE. 2, UnEc)
        CALL ParseAry(FileLines,    'WE_FOPoles',       CntrPar%WE_FOPoles,     CntrPar%WE_FOPoles_N,   accINFILE(1), ErrVar, CntrPar%WE_Mode .NE. 2, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        ! Retired WSE inputs:  not used anywhere in code
        ! CALL ParseInput(FileLines,  'WE_CP_n',accINFILE(1),CntrPar%WE_CP_n,ErrVar, UnEc)
        ! CALL ParseAry(  FileLines,  'WE_CP', CntrPar%WE_CP, CntrPar%WE_CP_n, accINFILE(1), ErrVar, .FALSE. , UnEc)

        !-------------- YAW CONTROLLER CONSTANTS -----------------
        CALL ParseInput(FileLines,  'Y_uSwitch',    CntrPar%Y_uSwitch,         accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseAry(  FileLines,  'Y_ErrThresh',  CntrPar%Y_ErrThresh,    2, accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'Y_Rate',       CntrPar%Y_Rate,            accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'Y_MErrSet',    CntrPar%Y_MErrSet,         accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'Y_IPC_IntSat', CntrPar%Y_IPC_IntSat,      accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'Y_IPC_KP',     CntrPar%Y_IPC_KP,          accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        CALL ParseInput(FileLines,  'Y_IPC_KI',     CntrPar%Y_IPC_KI,          accINFILE(1), ErrVar,    CntrPar%Y_ControlMode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------
        CALL ParseInput(FileLines,  'TRA_ExclSpeed',    CntrPar%TRA_ExclSpeed,          accINFILE(1),   ErrVar, CntrPar%TRA_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'TRA_ExclBand',     CntrPar%TRA_ExclBand,           accINFILE(1),   ErrVar, CntrPar%TRA_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'TRA_RateLimit',    CntrPar%TRA_RateLimit,          accINFILE(1),   ErrVar, CntrPar%TRA_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'FA_KI',            CntrPar%FA_KI,                  accINFILE(1),   ErrVar, CntrPar%TD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'FA_HPFCornerFreq', CntrPar%FA_HPFCornerFreq,       accINFILE(1),   ErrVar, CntrPar%TD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'FA_IntSat',        CntrPar%FA_IntSat,              accINFILE(1),   ErrVar, CntrPar%TD_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ PEAK SHAVING ------------
        CALL ParseInput(FileLines,  'PS_BldPitchMin_N', CntrPar%PS_BldPitchMin_N,                               accINFILE(1), ErrVar, CntrPar%PS_Mode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PS_WindSpeeds',    CntrPar%PS_WindSpeeds,      CntrPar%PS_BldPitchMin_N,   accINFILE(1), ErrVar, CntrPar%PS_Mode == 0, UnEc)
        CALL ParseAry(  FileLines,  'PS_BldPitchMin',   CntrPar%PS_BldPitchMin,     CntrPar%PS_BldPitchMin_N,   accINFILE(1), ErrVar, CntrPar%PS_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN
        
        !------------ STARTUP ------------
        CALL ParseInput(FileLines,  'SU_StartTime',             CntrPar%SU_StartTime,                                           accINFILE(1),   ErrVar, CntrPar%SU_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SU_FW_MinDuration',        CntrPar%SU_FW_MinDuration,                                      accINFILE(1),   ErrVar, CntrPar%SU_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SU_RotorSpeedThresh',      CntrPar%SU_RotorSpeedThresh,                                    accINFILE(1),   ErrVar, CntrPar%SU_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SU_RotorSpeedCornerFreq',  CntrPar%SU_RotorSpeedCornerFreq,                                accINFILE(1),   ErrVar, CntrPar%SU_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SU_LoadStages_N',          CntrPar%SU_LoadStages_N,                                        accINFILE(1),   ErrVar, CntrPar%SU_Mode == 0, UnEc)
        CALL ParseAry(FileLines,    'SU_LoadStages',            CntrPar%SU_LoadStages,              CntrPar%SU_LoadStages_N,    accINFILE(1),   ErrVar, CntrPar%SU_LoadStages_N == 0, UnEc)
        CALL ParseAry(FileLines,    'SU_LoadRampDuration',      CntrPar%SU_LoadRampDuration,        CntrPar%SU_LoadStages_N,    accINFILE(1),   ErrVar, CntrPar%SU_LoadStages_N == 0, UnEc)
        CALL ParseAry(FileLines,    'SU_LoadHoldDuration',      CntrPar%SU_LoadHoldDuration,        CntrPar%SU_LoadStages_N,    accINFILE(1),   ErrVar, CntrPar%SU_LoadStages_N == 0, UnEc)

        !------------ SHUTDOWN ------------
        CALL ParseInput(FileLines,  'SD_TimeActivate',       CntrPar%SD_TimeActivate,  accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_EnablePitch',        CntrPar%SD_EnablePitch,   accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_EnableYawError',     CntrPar%SD_EnableYawError,accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_EnableGenSpeed',     CntrPar%SD_EnableGenSpeed,accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_EnableTime',         CntrPar%SD_EnableTime,    accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_MaxPit',             CntrPar%SD_MaxPit,        accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_PitchCornerFreq',    CntrPar%SD_PitchCornerFreq,   accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_MaxYawError',        CntrPar%SD_MaxYawError,       accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_YawErrorCornerFreq', CntrPar%SD_YawErrorCornerFreq,accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_MaxGenSpd',          CntrPar%SD_MaxGenSpd,      accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_GenSpdCornerFreq',   CntrPar%SD_GenSpdCornerFreq,  accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_Time',               CntrPar%SD_Time,      accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_Method',             CntrPar%SD_Method,    accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'SD_Stage_N',            CntrPar%SD_Stage_N,    accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseAry(FileLines,    'SD_StageTime',         CntrPar%SD_StageTime,      CntrPar%SD_Stage_N,   accINFILE(1),   ErrVar, CntrPar%SD_Method .NE. 1, UnEc)
        CALL ParseAry(FileLines,    'SD_StagePitch',         CntrPar%SD_StagePitch,      CntrPar%SD_Stage_N,   accINFILE(1),   ErrVar, CntrPar%SD_Method .NE. 2, UnEc)
        CALL ParseAry(FileLines,    'SD_MaxTorqueRate',      CntrPar%SD_MaxTorqueRate,   CntrPar%SD_Stage_N,   accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        CALL ParseAry(FileLines,    'SD_MaxPitchRate',       CntrPar%SD_MaxPitchRate,    CntrPar%SD_Stage_N,   accINFILE(1),   ErrVar, CntrPar%SD_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ FLOATING ------------
        CALL ParseInput(FileLines,  'Fl_n',     CntrPar%Fl_n,                   accINFILE(1), ErrVar, .TRUE., UnEc)
        IF (CntrPar%Fl_n == 0) CntrPar%Fl_n = 1   ! Default is 1
        CALL ParseAry(FileLines,    'Fl_Kp',      CntrPar%Fl_Kp,  CntrPar%Fl_n,   accINFILE(1), ErrVar, CntrPar%FL_Mode == 0, UnEc)
        CALL ParseAry(FileLines,    'Fl_U',       CntrPar%Fl_U,  CntrPar%Fl_n,   accINFILE(1), ErrVar, CntrPar%Fl_n == 1, UnEc)  ! Allow default if only one Fl_Kp
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ Flaps ------------
        CALL ParseInput(FileLines, 'Flp_Angle',     CntrPar%Flp_Angle,      accINFILE(1), ErrVar,   CntrPar%Flp_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'Flp_Kp',        CntrPar%Flp_Kp,         accINFILE(1), ErrVar,   CntrPar%Flp_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'Flp_Ki',        CntrPar%Flp_Ki,         accINFILE(1), ErrVar,   CntrPar%Flp_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'Flp_MaxPit',    CntrPar%Flp_MaxPit,     accINFILE(1), ErrVar,   CntrPar%Flp_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ Open loop input ------------
        ! Indices can be left 0 by default, checked later
        CALL ParseInput(FileLines, 'OL_Filename',       CntrPar%OL_Filename,            accINFILE(1),   ErrVar, CntrPar%OL_Mode == 0,   UnEc)
        CALL ParseInput(FileLines, 'OL_BP_Mode',        CntrPar%OL_BP_Mode,             accINFILE(1),   ErrVar, CntrPar%OL_Mode == 0,   UnEc)
        CALL ParseInput(FileLines, 'OL_BP_FiltFreq',    CntrPar%OL_BP_FiltFreq,         accINFILE(1),   ErrVar, CntrPar%OL_Mode == 0,   UnEc)
        CALL ParseInput(FileLines, 'Ind_Breakpoint',    CntrPar%Ind_Breakpoint,         accINFILE(1),   ErrVar,                         UnEc=UnEc)
        CALL ParseAry(  FileLines, 'Ind_BldPitch',      CntrPar%Ind_BldPitch,       3,  accINFILE(1),   ErrVar,                         UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_GenTq',         CntrPar%Ind_GenTq,              accINFILE(1),   ErrVar,                         UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_YawRate',       CntrPar%Ind_YawRate,            accINFILE(1),   ErrVar,                         UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_Azimuth',       CntrPar%Ind_Azimuth,            accINFILE(1),   ErrVar, CntrPar%OL_Mode .NE. 2, UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_R_Speed',       CntrPar%Ind_R_Speed,            accINFILE(1),   ErrVar, CntrPar%OL_Mode .NE. 2, UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_R_Torque',      CntrPar%Ind_R_Torque,           accINFILE(1),   ErrVar, CntrPar%OL_Mode .NE. 2, UnEc=UnEc)
        CALL ParseInput(FileLines, 'Ind_R_Pitch',       CntrPar%Ind_R_Pitch,            accINFILE(1),   ErrVar, CntrPar%OL_Mode .NE. 2, UnEc=UnEc)
        CALL ParseAry(  FileLines, 'RP_Gains',          CntrPar%RP_Gains,           4,  accINFILE(1),   ErrVar, CntrPar%OL_Mode .NE. 2, UnEc=UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ Pitch Actuator Inputs ------------
        CALL ParseInput(FileLines, 'PA_CornerFreq',     CntrPar%PA_CornerFreq,  accINFILE(1),   ErrVar, CntrPar%PA_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'PA_Damping',        CntrPar%PA_Damping,     accINFILE(1),   ErrVar, CntrPar%PA_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ Pitch Actuator Faults ------------
        CALL ParseAry(FileLines,    'PF_Offsets',   CntrPar%PF_Offsets,     3,  accINFILE(1),    ErrVar, CntrPar%PF_Mode .NE. 1, UnEc)
        CALL ParseAry(FileLines,    'PF_TimeStuck', CntrPar%PF_TimeStuck,   3,  accINFILE(1),    ErrVar, CntrPar%PF_Mode .NE. 2, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ AWC input ------------
        CALL ParseInput(FileLines, 'AWC_NumModes',    CntrPar%AWC_NumModes,                           accINFILE(1), ErrVar, CntrPar%AWC_Mode == 0, UnEc)
        CALL ParseAry(  FileLines, 'AWC_n',           CntrPar%AWC_n,          CntrPar%AWC_NumModes,   accINFILE(1), ErrVar, CntrPar%AWC_Mode /= 1, UnEc)
        CALL ParseAry(  FileLines, 'AWC_harmonic',    CntrPar%AWC_harmonic,   CntrPar%AWC_NumModes,   accINFILE(1), ErrVar, CntrPar%AWC_Mode < 2,  UnEc)
        CALL ParseAry(  FileLines, 'AWC_freq',        CntrPar%AWC_freq,       CntrPar%AWC_NumModes,   accINFILE(1), ErrVar, CntrPar%AWC_Mode == 0, UnEc)
        CALL ParseAry(  FileLines, 'AWC_amp',         CntrPar%AWC_amp,        CntrPar%AWC_NumModes,   accINFILE(1), ErrVar, CntrPar%AWC_Mode == 0, UnEc)
        CALL ParseAry(  FileLines, 'AWC_clockangle',  CntrPar%AWC_clockangle, CntrPar%AWC_NumModes,   accINFILE(1), ErrVar, CntrPar%AWC_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'AWC_phaseoffset', CntrPar%AWC_phaseoffset,                        accINFILE(1), ErrVar, CntrPar%AWC_Mode == 0, UnEc)
        CALL ParseAry(  FileLines, 'AWC_CntrGains',   CntrPar%AWC_CntrGains,  2,                      accINFILE(1), ErrVar, CntrPar%AWC_Mode < 3, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ External control interface ------------
        CALL ParseInput(FileLines, 'DLL_FileName',  CntrPar%DLL_FileName,   accINFILE(1), ErrVar,   CntrPar%Ext_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'DLL_InFile',    CntrPar%DLL_InFile,     accINFILE(1), ErrVar,   CntrPar%Ext_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'DLL_ProcName',  CntrPar%DLL_ProcName,   accINFILE(1), ErrVar,   CntrPar%Ext_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------ ZeroMQ ------------
        CALL ParseInput(FileLines, 'ZMQ_ID',    CntrPar%ZMQ_ID,     accINFILE(1), ErrVar, .TRUE., UnEc=UnEc)
        CALL ParseInput(FileLines, 'ZMQ_CommAddress',   CntrPar%ZMQ_CommAddress,    accINFILE(1),   ErrVar,    CntrPar%ZMQ_Mode == 0, UnEc)
        CALL ParseInput(FileLines, 'ZMQ_UpdatePeriod',  CntrPar%ZMQ_UpdatePeriod,   accINFILE(1),   ErrVar,    CntrPar%ZMQ_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------- Cable Control ----- 
        CALL ParseInput(FileLines,  'CC_Group_N',    CntrPar%CC_Group_N,                        accINFILE(1), ErrVar, CntrPar%CC_Mode == 0, UnEc)
        CALL ParseAry( FileLines,   'CC_GroupIndex', CntrPar%CC_GroupIndex, CntrPar%CC_Group_N, accINFILE(1), ErrVar, CntrPar%CC_Mode == 0, UnEc)
        CALL ParseInput(FileLines,  'CC_ActTau',     CntrPar%CC_ActTau,                         accINFILE(1), ErrVar, CntrPar%CC_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        !------------- StC Control ----- 
        CALL ParseInput(FileLines,  'StC_Group_N',      CntrPar%StC_Group_N,                            accINFILE(1), ErrVar, CntrPar%StC_Mode == 0, UnEc)
        CALL ParseAry(  FileLines,  'StC_GroupIndex',   CntrPar%StC_GroupIndex, CntrPar%StC_Group_N,    accINFILE(1), ErrVar, CntrPar%StC_Mode == 0, UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        ! Open loop cable, structural control, needs number of groups
        CALL ParseAry(  FileLines, 'Ind_CableControl',   CntrPar%Ind_CableControl,  CntrPar%CC_Group_N,     accINFILE(1),   ErrVar,  CntrPar%CC_Mode .NE. 2,    UnEc=UnEc)
        CALL ParseAry(  FileLines, 'Ind_StructControl',  CntrPar%Ind_StructControl, CntrPar%StC_Group_N,    accINFILE(1),   ErrVar,  CntrPar%StC_Mode .NE. 2,   UnEc=UnEc)
        IF (ErrVar%aviFAIL < 0) RETURN

        IF (UnEc > 0) CLOSE(UnEc)     ! Close echo file

        !-------------------
        !------------------- CALCULATED CONSTANTS -----------------------
        !----------------------------------------------------------------

        ! Fix defaults manually for now
        IF (CntrPar%DT_Out == 0) THEN
            CntrPar%DT_Out = LocalVar%DT
        ENDIF

        ! DT_Out
        CntrPar%n_DT_Out = NINT(CntrPar%DT_Out / LocalVar%DT)
        CntrPar%n_DT_ZMQ = NINT(CntrPar%ZMQ_UpdatePeriod / LocalVar%DT)


        ! Fix Paths (add relative paths if called from another dir, UnEc)
        IF (PathIsRelative(CntrPar%PerfFileName)) CntrPar%PerfFileName = TRIM(PriPath)//TRIM(CntrPar%PerfFileName)
        IF (PathIsRelative(CntrPar%OL_Filename)) CntrPar%OL_Filename = TRIM(PriPath)//TRIM(CntrPar%OL_Filename)
        
        ! Convert yaw rate to deg/s
        CntrPar%Y_Rate = CntrPar%Y_Rate * R2D
        

        CntrPar%PC_RtTq99 = CntrPar%VS_RtTq*0.99
        CntrPar%VS_MinOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_MinOMSpd**2
        CntrPar%VS_MaxOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_RefSpd**2

        ! Read open loop input, if desired
        IF (CntrPar%OL_Mode > 0) THEN
            OL_String = ''      ! Display string
            OL_Count  = 1
            IF (CntrPar%Ind_BldPitch(1) > 0) THEN
                OL_String   = TRIM(OL_String)//' BldPitch1 '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_BldPitch(2) > 0) THEN
                OL_String   = TRIM(OL_String)//' BldPitch2 '
                ! If there are duplicate indices, don't increment OL_Count
                IF (.NOT. ((CntrPar%Ind_BldPitch(2) == CntrPar%Ind_BldPitch(1)) .OR. &
                (CntrPar%Ind_BldPitch(2) == CntrPar%Ind_BldPitch(3)))) THEN
                    OL_Count    = OL_Count + 1
                ENDIF
            ENDIF

            IF (CntrPar%Ind_BldPitch(3) > 0) THEN
                OL_String   = TRIM(OL_String)//' BldPitch3 '
                ! If there are duplicate indices, don't increment OL_Count
                IF (.NOT. ((CntrPar%Ind_BldPitch(3) == CntrPar%Ind_BldPitch(1)) .OR. &
                (CntrPar%Ind_BldPitch(3) == CntrPar%Ind_BldPitch(2)))) THEN
                    OL_Count    = OL_Count + 1
                ENDIF
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                OL_String   = TRIM(OL_String)//' GenTq '
                OL_Count    = OL_Count + 1  ! Read channel still, so we don't have issues
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                OL_String   = TRIM(OL_String)//' YawRate '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_Azimuth > 0) THEN
                IF (CntrPar%OL_Mode == 2) THEN
                    OL_String   = TRIM(OL_String)//' Azimuth '
                    OL_Count    = OL_Count + 1
                END IF
            ENDIF

            IF (CntrPar%Ind_R_Speed > 0) THEN
                IF (CntrPar%OL_Mode == 1) THEN
                    OL_String   = TRIM(OL_String)//' R_Speed '
                    OL_Count    = OL_Count + 1
                END IF
            ENDIF

            IF (CntrPar%Ind_R_Torque > 0) THEN
                IF (CntrPar%OL_Mode == 1) THEN
                    OL_String   = TRIM(OL_String)//' R_Torque '
                    OL_Count    = OL_Count + 1
                END IF
            ENDIF

            IF (CntrPar%Ind_R_Pitch > 0) THEN
                IF (CntrPar%OL_Mode == 1) THEN
                    OL_String   = TRIM(OL_String)//' R_Pitch '
                    OL_Count    = OL_Count + 1
                END IF
            ENDIF

            N_OL_Cables = 0
            IF (ANY(CntrPar%Ind_CableControl > 0)) THEN
                DO I = 1,SIZE(CntrPar%Ind_CableControl)
                    IF (CntrPar%Ind_CableControl(I) > 0) THEN
                        OL_String   = TRIM(OL_String)//' Cable'//TRIM(Int2LStr(I))//' '
                        OL_Count    = OL_Count + 1
                        N_OL_Cables = N_OL_Cables + 1
                    ENDIF
                ENDDO
            ENDIF

            N_OL_StCs = 0
            IF (ANY(CntrPar%Ind_StructControl > 0)) THEN
                DO I = 1,SIZE(CntrPar%Ind_StructControl)
                    IF (CntrPar%Ind_StructControl(I) > 0) THEN
                        OL_String   = TRIM(OL_String)//' StC'//TRIM(Int2LStr(I))//' '
                        OL_Count    = OL_Count + 1
                        N_OL_StCs   = N_OL_StCs + 1
                    ENDIF
                ENDDO
            ENDIF


            PRINT *, 'ROSCO: Implementing open loop control for'//TRIM(OL_String)
            IF (CntrPar%OL_Mode == 2) THEN
                PRINT *, 'ROSCO: OL_Mode = 2 will change generator torque control for Azimuth tracking'
            ENDIF

            CALL GetNewUnit(UnOpenLoop, ErrVar)
            CALL Read_OL_Input(CntrPar%OL_Filename,UnOpenLoop,OL_Count,CntrPar%OL_Channels, ErrVar)
            IF (ErrVar%aviFAIL < 0) THEN
                RETURN
            ENDIF

            CntrPar%OL_Breakpoints = CntrPar%OL_Channels(:,CntrPar%Ind_Breakpoint)

            ! Set OL Inputs based on indices
            IF (CntrPar%Ind_BldPitch(1) > 0) THEN
                CntrPar%OL_BldPitch1 = CntrPar%OL_Channels(:,CntrPar%Ind_BldPitch(1))
            ENDIF

            IF (CntrPar%Ind_BldPitch(2) > 0) THEN
                CntrPar%OL_BldPitch2 = CntrPar%OL_Channels(:,CntrPar%Ind_BldPitch(2))
            ENDIF

            IF (CntrPar%Ind_BldPitch(3) > 0) THEN
                CntrPar%OL_BldPitch3 = CntrPar%OL_Channels(:,CntrPar%Ind_BldPitch(3))
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                CntrPar%OL_GenTq = CntrPar%OL_Channels(:,CntrPar%Ind_GenTq)
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                CntrPar%OL_YawRate = CntrPar%OL_Channels(:,CntrPar%Ind_YawRate)
            ENDIF

            IF (CntrPar%Ind_Azimuth > 0) THEN
                CntrPar%OL_Azimuth = Unwrap(CntrPar%OL_Channels(:,CntrPar%Ind_Azimuth),ErrVar)
            ENDIF

            IF (CntrPar%Ind_R_Speed > 0) THEN
                CntrPar%OL_R_Speed = CntrPar%OL_Channels(:,CntrPar%Ind_R_Speed)
            ENDIF

            IF (CntrPar%Ind_R_Torque > 0) THEN
                CntrPar%OL_R_Torque = CntrPar%OL_Channels(:,CntrPar%Ind_R_Torque)
            ENDIF

            IF (CntrPar%Ind_R_Pitch > 0) THEN
                CntrPar%OL_R_Pitch = CntrPar%OL_Channels(:,CntrPar%Ind_R_Pitch)
            ENDIF
            
            IF (ANY(CntrPar%Ind_CableControl > 0)) THEN
                ALLOCATE(CntrPar%OL_CableControl(N_OL_Cables,SIZE(CntrPar%OL_Channels,DIM=1)))
                I_OL = 1
                DO I = 1,SIZE(CntrPar%Ind_CableControl)
                    IF (CntrPar%Ind_CableControl(I) > 0) THEN
                        CntrPar%OL_CableControl(I_OL,:) = CntrPar%OL_Channels(:,CntrPar%Ind_CableControl(I))
                        I_OL = I_OL + 1
                    ENDIF
                ENDDO
            ENDIF

            IF (ANY(CntrPar%Ind_StructControl > 0)) THEN
                ALLOCATE(CntrPar%OL_StructControl(N_OL_StCs,SIZE(CntrPar%OL_Channels,DIM=1)))
                I_OL = 1
                DO I = 1,SIZE(CntrPar%Ind_StructControl)
                    IF (CntrPar%Ind_StructControl(I) > 0) THEN
                        CntrPar%OL_StructControl(I_OL,:) = CntrPar%OL_Channels(:,CntrPar%Ind_StructControl(I))
                        I_OL = I_OL + 1
                    ENDIF
                ENDDO
            ENDIF

        END IF

        
        !------------------- HOUSEKEEPING -----------------------
        CntrPar%PerfFileName = TRIM(CntrPar%PerfFileName)
        

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF


    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData, ErrVar)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters, ErrorVariables
        
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar
        
        ! Local variables
        INTEGER(IntKi)                                  :: UnPerfParameters
        INTEGER(IntKi)                                  :: i ! iteration index

        INTEGER(IntKi)                                  :: CurLine 
        INTEGER                                         :: IOS   
        CHARACTER(256)                                  :: IOM
        CHARACTER(*), PARAMETER                         :: RoutineName = 'ReadCpFile'
        REAL(DbKi), DIMENSION(:), ALLOCATABLE           :: TmpPerf

        CurLine = 1
        CALL GetNewUnit(UnPerfParameters, ErrVar)
        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read')
                
        ! ----------------------- Axis Definitions ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ParseAry(UnPerfParameters, CurLine, 'Pitch angle vector', PerfData%Beta_vec, CntrPar%PerfTableSize(1), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ParseAry(UnPerfParameters, CurLine, 'TSR vector', PerfData%TSR_vec, CntrPar%PerfTableSize(2), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) ! Input file should contains wind speed information here - unneeded for now
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Cp_mat(i,:) ! Read Cp table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table, IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Ct_mat(i,:) ! Read Ct table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table, IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Cq_mat(i,:) ! Read Cq table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO

        ! Close file
        CLOSE(UnPerfParameters)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
    
    END SUBROUTINE ReadCpFile
    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution
    SUBROUTINE CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
    
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)       :: LocalVar
        TYPE(ErrorVariables),       INTENT(INOUT)       :: ErrVar
        INTEGER(IntKi),                 INTENT(IN   )       :: size_avcMSG
        INTEGER(IntKi)                                  :: Imode       ! Index used for looping through AWC modes
        REAL(ReKi),              INTENT(IN   )       :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        CHARACTER(*), PARAMETER                         :: RoutineName = 'CheckInputs'
        ! Local

        INTEGER(IntKi)                                  :: I
        INTEGER(IntKi), ALLOCATABLE                     :: All_OL_Indices(:)
        
        !..............................................................................................................................
        ! Check validity of input parameters:
        !..............................................................................................................................

        !------- DEBUG ------------------------------------------------------------

        ! LoggingLevel
        IF ((CntrPar%LoggingLevel < 0) .OR. (CntrPar%LoggingLevel > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'LoggingLevel must be 0 - 3.'
        ENDIF

        IF (CntrPar%DT_Out .le. 0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'DT_Out must be greater than 0'
        ENDIF

        IF (CntrPar%DT_Out < LocalVar%DT) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'DT_Out must be greater than or equal to DT in OpenFAST'
        ENDIF

        IF (ABS(CntrPar%DT_out - Localvar%DT * CntrPar%n_DT_Out) > 0.001_DbKi) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'DT_Out must be a factor of DT in OpenFAST'
        ENDIF

        IF (CntrPar%ZMQ_Mode > 0) THEN
            IF (ABS(CntrPar%ZMQ_UpdatePeriod - Localvar%DT * CntrPar%n_DT_ZMQ) > 0.001_DbKi) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'ZMQ_UpdatePeriod must be a factor of DT in OpenFAST'
            ENDIF
        ENDIF
        !------- CONTROLLER FLAGS -------------------------------------------------

        ! F_LPFType
        IF ((CntrPar%F_LPFType < 1) .OR. (CntrPar%F_LPFType > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_LPFType must be 1 or 2.'
        ENDIF

        ! IPC_ControlMode
        IF ((CntrPar%IPC_ControlMode < 0) .OR. (CntrPar%IPC_ControlMode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_ControlMode must be 0, 1, or 2.'
        ENDIF

        ! VS_ControlMode
        IF ((CntrPar%VS_ControlMode < 0) .OR. (CntrPar%VS_ControlMode > 4)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_ControlMode must be between 0 and 4.'
        ENDIF

        ! VS_ConstPower
        IF ((CntrPar%VS_ConstPower < 0) .OR. (CntrPar%VS_ConstPower > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_ConstPower must be 0 or 1.'
        ENDIF

        ! VS_FBP
        IF ((CntrPar%VS_FBP < 0) .OR. (CntrPar%VS_FBP > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_FBP must be between 0 and 3.'
        ENDIF
        IF ((CntrPar%VS_FBP > 0) .AND. (CntrPar%PC_ControlMode > 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_FBP and PC_ControlMode cannot both be greater than 0.'
        ENDIF

        IF ((CntrPar%VS_FBP > 0) .AND. (CntrPar%PRC_Mode > 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Fixed blade pitch control (VS_FBP) and power reference control (PRC_Mode) cannot both be enabled.'
        ENDIF

        IF ((CntrPar%VS_FBP > 0) .AND. (CntrPar%VS_ConstPower > 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Fixed blade pitch control (VS_FBP) and constant power torque control (VS_ConstPower) cannot both be enabled.'
        ENDIF

        ! PC_ControlMode
        IF ((CntrPar%PC_ControlMode < 0) .OR. (CntrPar%PC_ControlMode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_ControlMode must be 0 or 1.'
        ENDIF

        ! Y_ControlMode
        IF ((CntrPar%Y_ControlMode < 0) .OR. (CntrPar%Y_ControlMode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Y_ControlMode must be 0, 1 or 2.'
        ENDIF

        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Y_ControlMode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously'
        ENDIF

        ! SS_Mode
        IF ((CntrPar%SS_Mode < 0) .OR. (CntrPar%SS_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_Mode must be 0 or 1.'
        ENDIF

        ! WE_Mode
        IF ((CntrPar%WE_Mode < 0) .OR. (CntrPar%WE_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_Mode must be 0, 1, or 2.'
        ENDIF

        ! PS_Mode
        IF ((CntrPar%PS_Mode < 0) .OR. (CntrPar%PS_Mode > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PS_Mode must be 0 or 1.'
        ENDIF
        
        ! SU_Mode
        IF ((CntrPar%SU_Mode < 0) .OR. (CntrPar%SU_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SU_Mode must be 0 or 1.'
        ENDIF

        ! SD_Mode
        IF ((CntrPar%SD_Mode < 0) .OR. (CntrPar%SD_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SD_Mode must be 0 or 1.'
        ENDIF

        ! Fl_Mode
        IF ((CntrPar%Fl_Mode < 0) .OR. (CntrPar%Fl_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Fl_Mode must be 0, 1, or 2.'
        ENDIF

        ! Flp_Mode
        IF ((CntrPar%Flp_Mode < 0) .OR. (CntrPar%Flp_Mode > 3)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Flp_Mode must be 0, 1, 2, or 3.'
        ENDIF

        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Flp_Mode > 0)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg   = 'ROSCO does not currently support IPC_ControlMode and Flp_Mode > 0'
        ENDIF
        !------- FILTERS ----------------------------------------------------------
        
        ! F_LPFCornerFreq
        IF (CntrPar%F_LPFCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_LPFCornerFreq must be greater than zero.'
        ENDIF

        ! F_LPFDamping
        IF (CntrPar%F_LPFType == 2) THEN
            IF (CntrPar%F_LPFDamping <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_LPFDamping must be greater than zero.'
            ENDIF
        ENDIF

        ! Notch Filter Params
        IF (CntrPar%F_NumNotchFilts > 0) THEN

            ! F_NotchCornerFreq
            IF (ANY(CntrPar%F_NotchFreqs <= 0.0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_NotchFreqs must be greater than zero.'
            ENDIF

            ! F_NotchBetaDen
            IF (ANY(CntrPar%F_NotchBetaDen <= 0.0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_NotchBetaDen must be greater than zero.'
            ENDIF
        ENDIF

        ! F_SSCornerFreq
        IF (CntrPar%F_SSCornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_SSCornerFreq must be greater than zero.'
        ENDIF

        ! F_WECornerFreq
        IF (CntrPar%F_WECornerFreq <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'F_WECornerFreq must be greater than zero.'
        ENDIF

        IF (CntrPar%Fl_Mode > 0) THEN
            ! F_FlCornerFreq(1)  (frequency)
            IF (CntrPar%F_FlCornerFreq(1) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlCornerFreq(1) must be greater than zero.'
            ENDIF

            ! F_FlCornerFreq(2)  (damping)
            IF (CntrPar%F_FlCornerFreq(2) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlCornerFreq(2) must be greater than zero.'
            ENDIF

            ! F_FlHighPassFreq
            IF (CntrPar%F_FlHighPassFreq <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlHighPassFreq must be greater than zero.'
            ENDIF
        ENDIF

        IF (CntrPar%Flp_Mode > 0) THEN
            ! F_FlpCornerFreq(1)  (frequency)
            IF (CntrPar%F_FlpCornerFreq(1) <= 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlpCornerFreq(1) must be greater than zero.'
            ENDIF

            ! F_FlpCornerFreq(2)  (damping)
            IF (CntrPar%F_FlpCornerFreq(2) < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'F_FlpCornerFreq(2) must be greater than or equal to zero.'
            ENDIF
        ENDIF
                     
        
        !------- BLADE PITCH CONTROL ----------------------------------------------

        ! PC_GS_n
        IF (CntrPar%PC_GS_n <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_GS_n must be greater than 0'
        ENDIF

        ! PC_GS_angles
        IF (CntrPar%PC_ControlMode .NE. 0 .AND. .NOT. NonDecreasing(CntrPar%PC_GS_angles)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_GS_angles must be non-decreasing'
        ENDIF

        ! PC_GS_KP and PC_GS_KI
        ! I'd like to throw warnings if these are positive

        ! PC_MinPit and PC_MaxPit
        IF (CntrPar%PC_MinPit >= CntrPar%PC_MaxPit)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
        ENDIF

        ! PC_RefSpd
        IF (CntrPar%PC_RefSpd <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_RefSpd must be greater than zero.'
        ENDIF
        
        ! PC_MaxRat
        IF (CntrPar%PC_MaxRat <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MaxRat must be greater than zero.'
        ENDIF

        ! PC_MinRat
        IF (CntrPar%PC_MinRat >= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PC_MinRat must be less than zero.'
        ENDIF

        !------- INDIVIDUAL PITCH CONTROL -----------------------------------------

        IF (CntrPar%IPC_CornerFreqAct < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Corner frequency of IPC actuator model must be positive, or set to 0 to disable.'
        ENDIF

        IF (CntrPar%IPC_SatMode < 0 .OR. CntrPar%IPC_SatMode > 3)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_SatMode must be 0, 1, 2, or 3.'
        ENDIF

        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KI(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KI(2) must be zero or greater than zero.'
        ENDIF

        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KP(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC_KP(2) must be zero or greater than zero.'
        ENDIF

        !------- VS TORQUE CONTROL ------------------------------------------------
       
        IF (CntrPar%VS_MaxRat <= 0.0) THEN
            ErrVar%aviFAIL =  -1
            ErrVar%ErrMsg  = 'VS_MaxRat must be greater than zero.'
        ENDIF
        
        
        
        ! VS_Rgn2K
        IF (CntrPar%VS_Rgn2K < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_Rgn2K must not be negative.'
        ENDIF
        
        ! VS_RtTq
        IF (CntrPar%VS_MaxTq < CntrPar%VS_RtTq) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
        ENDIF

        ! VS_RtPwr
        IF (CntrPar%VS_RtPwr < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtPwr must not be negative.'
        ENDIF

        ! VS_RtTq
        IF (CntrPar%VS_RtTq < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_RtTq must not be negative.'
        ENDIF

        ! VS_KP
        IF (CntrPar%VS_KP(1) > 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_KP must be less than zero.'
        ENDIF
        
        ! VS_KI
        IF (CntrPar%VS_KI(1) > 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_KI must be less than zero.'
        ENDIF

        ! VS_TSRopt
        IF (CntrPar%VS_TSRopt < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'VS_TSRopt must be greater than zero.'
        ENDIF
        
        !------- SETPOINT SMOOTHER ---------------------------------------------

        ! SS_VSGain
        IF (CntrPar%SS_VSGain < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_VSGain must be greater than zero.'
        ENDIF

        ! SS_PCGain
        IF (CntrPar%SS_PCGain < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'SS_PCGain must be greater than zero.'
        ENDIF

        IF ((CntrPar%PRC_Mode < 0) .OR. (CntrPar%PRC_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PRC_Mode must be 0, 1, or 2.'
        ENDIF

        IF (CntrPar%PRC_Mode == 2) THEN
            PRINT *, "Note: PRC Mode = ", CntrPar%PRC_Mode, ", which will affect VS_RefSpeed, VS_TSRopt, and PC_RefSpeed"

            IF (CntrPar%PRC_Comm == 0) THEN
                IF (CntrPar%PRC_R_Pitch < 0) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'PRC_R_Pitch must be greater than or equal to zero.'
                ENDIF

                IF (CntrPar%PRC_R_Speed < 0) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'PRC_R_Speed must be greater than or equal to zero.'
                ENDIF

                IF (CntrPar%PRC_R_Torque < 0) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'PRC_R_Torque must be greater than or equal to zero.'
                ENDIF

            ENDIF

            IF ((CntrPar%PRC_Comm == 1) .AND. (CntrPar%OL_Mode .NE. 1)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'OL_Mode must be 1 to use open loop inputs for power control (PRC_Comm = 1).'
            ENDIF

            IF ((CntrPar%PRC_Comm == 2) .AND. (CntrPar%ZMQ_Mode .NE. 1)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'ZMQ_Mode must be 1 to use ZeroMQ inputs for power control (PRC_Comm = 2).'
            ENDIF

        ENDIF
        
        !------- WIND SPEED ESTIMATOR ---------------------------------------------

        ! WE_BladeRadius
        IF (CntrPar%WE_BladeRadius < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_BladeRadius must be greater than zero.'
        ENDIF

        ! WE_GearboxRatio
        IF (CntrPar%WE_GearboxRatio < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_GearboxRatio must be greater than zero.'
        ENDIF

        ! WE_Jtot
        IF (CntrPar%WE_Jtot < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_Jtot must be greater than zero.'
        ENDIF

        ! WE_RhoAir
        IF (CntrPar%WE_RhoAir < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_RhoAir must be greater than zero.'
        ENDIF

        ! PerfTableSize(1)
        IF (CntrPar%PerfTableSize(1) < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PerfTableSize(1) must be greater than zero.'
        ENDIF

        ! PerfTableSize(2)
        IF (CntrPar%PerfTableSize(2) < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'PerfTableSize(2) must be greater than zero.'
        ENDIF

        ! WE_FOPoles_N
        IF (CntrPar%WE_FOPoles_N < 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_FOPoles_N must be greater than zero.'
        ENDIF

        ! WE_FOPoles_v
        IF (CntrPar%WE_Mode == 2 .AND. .NOT. NonDecreasing(CntrPar%WE_FOPoles_v)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'WE_FOPoles_v must be non-decreasing.'
        ENDIF

        ! ---- Yaw Control ----
        IF (CntrPar%Y_ControlMode > 0) THEN
            IF (CntrPar%Y_ControlMode == 1) THEN
                IF (CntrPar%Y_ErrThresh(1) <= 0.0)  THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'Y_ErrThresh must be greater than zero.'
                ENDIF

                IF (CntrPar%Y_Rate <= 0.0)  THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'CntrPar%Y_Rate must be greater than zero.'
                ENDIF
            ENDIF
        ENDIF

        ! ---- Tower Control ----
        IF (CntrPar%TD_Mode < 0 .OR. CntrPar%TD_Mode > 1) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'TD_Mode must be 0 or 1.'
        END IF

        IF (CntrPar%TRA_Mode < 0 .OR. CntrPar%TRA_Mode > 1) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'TRA_Mode must be 0 or 1.'
        END IF

        IF (CntrPar%TRA_Mode > 1) THEN  ! Frequency avoidance is active
            IF (CntrPar%TRA_ExclSpeed < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'TRA_ExclSpeed must be greater than 0.'
            END IF

            IF (CntrPar%TRA_ExclBand < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'TRA_ExclBand must be greater than 0.'
            END IF

            IF (CntrPar%TRA_RateLimit < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'TRA_RateLimit must be greater than 0.'
            END IF

            IF ( .NOT. ((CntrPar%VS_ControlMode == VS_Mode_WSE_TSR) .OR. (CntrPar%VS_ControlMode == VS_Mode_Power_TSR) )) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'VS_ControlMode must be 2 or 3 to use frequency avoidance control.'
            END IF

            IF (CntrPar%PRC_Mode == 1) THEN
                PRINT *, "ROSCO Warning: Note that frequency avoidance control (TRA_Mode > 1) will affect PRC set points"
            END IF           

        END IF

        !------- MINIMUM PITCH SATURATION -------------------------------------------
        IF (CntrPar%PS_Mode > 0) THEN

            ! PS_BldPitchMin_N
            IF (CntrPar%PS_BldPitchMin_N < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'PS_BldPitchMin_N must be greater than zero.'
            ENDIF

            ! PS_WindSpeeds
            IF (.NOT. NonDecreasing(CntrPar%PS_WindSpeeds)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'PS_WindSpeeds must be non-decreasing.'
            ENDIF


        ENDIF

        ! --- Startup ---
        IF (CntrPar%SU_Mode > 0) THEN

            ! SU_FW_MinDuration
            IF (CntrPar%SU_FW_MinDuration < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SU_FW_MinDuration must be greater than zero.'
            ENDIF

            ! SU_RotorSpeedThresh
            IF (CntrPar%SU_RotorSpeedThresh < 0.0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SU_RotorSpeedThresh must be greater than zero.'
            ENDIF
            
            ! SU_RotorSpeedCornerFreq
            IF (CntrPar%SU_RotorSpeedCornerFreq < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'SU_RotorSpeedCornerFreq must be greater than or equal to 0.'
            ENDIF
            
            ! SU_LoadStages_N
            IF (CntrPar%SU_LoadStages_N < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'SU_LoadStages_N must be greater than or equal to 0.'
            ENDIF

            ! SU_LoadStages
            IF (ANY(CntrPar%SU_LoadStages < 0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SU_LoadStages must be positive.'
            ENDIF

            ! SU_LoadRampDuration
            IF (ANY(CntrPar%SU_LoadRampDuration < 0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SU_LoadRampDuration must be positive.'
            ENDIF

            ! SU_LoadHoldDuration
            IF (ANY(CntrPar%SU_LoadHoldDuration < 0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SU_LoadHoldDuration must be positive.'
            ENDIF
        ENDIF

        ! --- Shutdown ---
        IF (CntrPar%SD_Mode > 0) THEN
        
            ! SD_Method
            IF (CntrPar%SD_Method < 1 .OR. CntrPar%SD_Method > 2) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SD_Method must be 1 or 2.'
            ENDIF

            ! SD_MaxPitchRate
            IF (MAXVAL(CntrPar%SD_MaxPitchRate) > CntrPar%PC_MaxRat) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SD_MaxPitchRate(s) should be less than or equal to PC_MaxRat.'
            ENDIF

                
            ! SD_MaxTorqueRate
            IF (MAXVAL(CntrPar%SD_MaxTorqueRate) > CntrPar%VS_MaxRat) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SD_MaxTorqueRate(s) should be less than or equal to VS_MaxRat.'
            ENDIF

            IF (CntrPar%SD_Stage_N < 1) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = 'SD_Stage_N must be greater than or equal to 1.'
            ENDIF

            IF (CntrPar%SD_Method == 1) THEN
                ! SD_StageTime must be greater than zero
                IF ( MINVAL(CntrPar%SD_StageTime) < 0.0) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'SD_StageTime(s) must be greater than or equal to zero.'
                ENDIF

            ELSEIF (CntrPar%SD_Method == 2) THEN
                ! SD_StagePitch must be increasing
                IF (.NOT. NonDecreasing(CntrPar%SD_StagePitch)) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg  = 'SD_StagePitch must be non-decreasing.'
                ENDIF

            ENDIF
        ENDIF

        ! --- Open loop control ---
        IF (CntrPar%OL_Mode > 0) THEN
            ! Get all open loop indices
            ALLOCATE(All_OL_Indices(5))   ! Will need to increase to 5 when IPC
            All_OL_Indices =    (/CntrPar%Ind_BldPitch, & 
                                CntrPar%Ind_GenTq, &
                                CntrPar%Ind_YawRate, &
                                CntrPar%Ind_R_Speed, &
                                CntrPar%Ind_R_Torque, &
                                CntrPar%Ind_R_Pitch/)

            DO I = 1,SIZE(CntrPar%Ind_CableControl)
                Call AddToList(All_OL_Indices, CntrPar%Ind_CableControl(I))           
            ENDDO
            
            DO I = 1,SIZE(CntrPar%Ind_StructControl)
                Call AddToList(All_OL_Indices, CntrPar%Ind_StructControl(I))             
            ENDDO       

            IF (ANY(All_OL_Indices < 0)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'All open loop control indices must be greater than zero'
            ENDIF

            IF (CntrPar%Ind_Breakpoint < 1) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'Ind_Breakpoint must be non-zero if OL_Mode is non-zero'
            ENDIF

            IF (ALL(All_OL_Indices < 1)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'At least one open loop input channel must be non-zero'
            ENDIF

            IF (CntrPar%OL_Mode == 2) THEN
                IF ((CntrPar%Ind_BldPitch(1) == 0) .OR. &
                (CntrPar%Ind_BldPitch(2) == 0) .OR. &
                (CntrPar%Ind_BldPitch(3) == 0) .OR. &
                (CntrPar%Ind_GenTq == 0) .OR. &
                (CntrPar%Ind_Azimuth == 0)) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = 'If OL_Mode = 2, Ind_BldPitch, Ind_GenTq, and Ind_Azimuth must be greater than zero'
                ENDIF
            ENDIF

            IF (ANY(CntrPar%Ind_CableControl > 0) .AND. CntrPar%CC_Mode .NE. 2) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'CC_Mode must be 2 if using open loop cable control via Ind_CableControl'
            ENDIF

            IF (ANY(CntrPar%Ind_StructControl > 0) .AND. CntrPar%StC_Mode .NE. 2) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'StC_Mode must be 2 if using open loop struct control via Ind_StructControl'
            ENDIF

            IF ((CntrPar%OL_BP_Mode < 0) .OR. (CntrPar%OL_BP_Mode > 1)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'OL_BP_Mode must be 0 or 1.'
            ENDIF

            IF (CntrPar%OL_BP_FiltFreq < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'OL_BP_FiltFreq must be greater than or equal to 0.'
            ENDIF

            IF ((CntrPar%OL_BP_Mode == 1) .AND. (CntrPar%OL_Mode == 2)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'Rotor position control (OL_Mode = 2) is not compatible with wind speed breakpoints (OL_BP_Mode = 1)'
            ENDIF

        ENDIF

        ! ---- AWC vs. IPC
        IF (CntrPar%AWC_Mode > 0 .AND. CntrPar%IPC_ControlMode > 0) THEN
            PRINT *, "ROSCO WARNING: Individual pitch control and active wake control are both enabled. Performance may be compromised."
        ENDIF



        ! --- Pitch Actuator ---
        IF (CntrPar%PA_Mode > 0) THEN
            IF ((CntrPar%PA_Mode < 0) .OR. (CntrPar%PA_Mode > 2)) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'PA_Mode must be 0, 1, or 2'
            END IF
            IF (CntrPar%PA_CornerFreq < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'PA_CornerFreq must be greater than 0'
            END IF
            IF (CntrPar%PA_Damping < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'PA_Damping must be greater than 0'
            END IF
        END IF

        ! --- Active Wake Control ---
        IF (CntrPar%AWC_Mode > 0) THEN
            IF (CntrPar%AWC_NumModes < 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'AWC_NumModes must be a positive integer if AWC_Mode = 1'
            END IF
            DO Imode = 1,CntrPar%AWC_NumModes
                IF (CntrPar%AWC_freq(Imode) < 0.0) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = 'AWC_freq cannot be less than 0'
                END IF
            END DO
            IF (CntrPar%AWC_Mode == 1) THEN
                DO Imode = 1,CntrPar%AWC_NumModes
                    IF ((CntrPar%AWC_clockangle(Imode) > 360.0) .OR. (CntrPar%AWC_clockangle(Imode) < 0.0)) THEN
                        ErrVar%aviFAIL = -1
                        ErrVar%ErrMsg = 'AWC_clockangle must be between 0 and 360 in AWC_Mode = 1'
                    END IF
                END DO    
            END IF

            IF (CntrPar%AWC_Mode == 2) THEN
                IF ((CntrPar%AWC_NumModes > 2) .OR. (CntrPar%AWC_NumModes < 1)) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = 'AWC_NumModes must be either 1 or 2 if AWC_Mode = 2'
                END IF
                DO Imode = 1,CntrPar%AWC_NumModes
                    IF ((CntrPar%AWC_clockangle(Imode) > 360.0) .OR. (CntrPar%AWC_clockangle(Imode) < -360.0)) THEN
                        ErrVar%aviFAIL = -1
                        ErrVar%ErrMsg = 'AWC_clockangle must be between -360 and 360 in AWC_Mode = 2'
                    END IF
                    IF (CntrPar%AWC_harmonic(Imode) < 0) THEN
                        ErrVar%aviFAIL = -1
                        ErrVar%ErrMsg = 'AWC_harmonic must be a positive integer'
                    END IF
                END DO
            END IF
        END IF

        IF ((CntrPar%CC_Mode < 0) .OR. (CntrPar%CC_Mode > 2)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = 'CC_Mode must be 0 or 1'
        END IF

        IF (CntrPar%CC_Mode > 0) THEN

            ! Extended avrSWAP must be used
            IF (CntrPar%Ext_Interface == 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON'
            ENDIF

            IF (CntrPar%CC_ActTau .LE. 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'CC_ActTau must be greater than 0.'
            END IF

            DO I = 1,CntrPar%CC_Group_N
                IF (CntrPar%CC_GroupIndex(I) < 2601) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = 'CC_GroupIndices must be greater than 2601.'        !< Starting index for the cable control
                END IF
            END DO
        END IF

        IF (CntrPar%StC_Mode > 0) THEN

            ! Extended avrSWAP must be used
            IF (CntrPar%Ext_Interface == 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON'
            ENDIF

            ! Check indices
            DO I = 1,CntrPar%StC_Group_N
                IF (CntrPar%StC_GroupIndex(I) < 2801) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = 'StC_GroupIndices must be greater than 2801.'        !< Starting index for the cable control
                END IF
            END DO
        END IF

        ! Check that open loop control active if using open loop cable/struct control
        IF (CntrPar%CC_Mode == 2 .AND. CntrPar%OL_Mode .NE. 1) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = 'OL_Mode must be 1 if using CC_Mode = 2 (open loop)'
        END IF

        IF (CntrPar%StC_Mode == 2 .AND. CntrPar%OL_Mode .NE. 1) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = 'OL_Mode must be 1 if using StC_Mode = 2 (open loop)'
        END IF
            

        
        ! Abort if the user has not requested a pitch angle actuator (See Appendix A
        ! of Bladed User's Guide):
        IF (NINT(avrSWAP(10)) /= 0)  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Pitch angle actuator not requested.'
        ENDIF
        
        IF ((NINT(avrSWAP(28)) == 0) .AND. &
            ((CntrPar%IPC_ControlMode > 0) .OR. &
             (CntrPar%Y_ControlMode > 1) .OR. & 
             (CntrPar%Ind_BldPitch(2) > 0) .OR. &
             (CntrPar%Ind_BldPitch(3) > 0) &
             )) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.'
        ENDIF

        ! PF_Mode = 1
        IF (NINT(avrSWAP(28)) == 0 .AND. (CntrPar%PF_Mode == 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'Pitch offset fault enabled (PF_Mode = 1), but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.'
        ENDIF

        IF (NINT(avrSWAP(28)) == 0 .AND. (CntrPar%AWC_Mode > 1)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'AWC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.'
        ENDIF

        ! DT
        IF (LocalVar%DT <= 0.0) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = 'DT must be greater than zero.'
        ENDIF

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE CheckInputs
    
END MODULE ReadSetParameters
