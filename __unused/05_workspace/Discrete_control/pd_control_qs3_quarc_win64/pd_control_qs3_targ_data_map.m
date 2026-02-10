    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (pd_control_qs3_P)
        ;%
            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% pd_control_qs3_P.ProportionalGain_gain
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.DerivativeGain_gain
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILWrite_analog_channels
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.HILWrite_other_channels
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 27;
            section.data(27)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILInitialize_OOTerminate
                    section.data(1).logicalSrcIdx = 4;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.HILInitialize_OOExit
                    section.data(2).logicalSrcIdx = 5;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_P.HILInitialize_OOStart
                    section.data(3).logicalSrcIdx = 6;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_P.HILInitialize_OOEnter
                    section.data(4).logicalSrcIdx = 7;
                    section.data(4).dtTransOffset = 3;

                    ;% pd_control_qs3_P.HILInitialize_AOFinal
                    section.data(5).logicalSrcIdx = 8;
                    section.data(5).dtTransOffset = 4;

                    ;% pd_control_qs3_P.HILInitialize_POFinal
                    section.data(6).logicalSrcIdx = 9;
                    section.data(6).dtTransOffset = 5;

                    ;% pd_control_qs3_P.HILInitialize_OOFinal
                    section.data(7).logicalSrcIdx = 10;
                    section.data(7).dtTransOffset = 6;

                    ;% pd_control_qs3_P.HILInitialize_AIHigh
                    section.data(8).logicalSrcIdx = 11;
                    section.data(8).dtTransOffset = 9;

                    ;% pd_control_qs3_P.HILInitialize_AILow
                    section.data(9).logicalSrcIdx = 12;
                    section.data(9).dtTransOffset = 10;

                    ;% pd_control_qs3_P.HILInitialize_AOHigh
                    section.data(10).logicalSrcIdx = 13;
                    section.data(10).dtTransOffset = 11;

                    ;% pd_control_qs3_P.HILInitialize_AOLow
                    section.data(11).logicalSrcIdx = 14;
                    section.data(11).dtTransOffset = 12;

                    ;% pd_control_qs3_P.HILInitialize_AOInitial
                    section.data(12).logicalSrcIdx = 15;
                    section.data(12).dtTransOffset = 13;

                    ;% pd_control_qs3_P.HILInitialize_AOWatchdog
                    section.data(13).logicalSrcIdx = 16;
                    section.data(13).dtTransOffset = 14;

                    ;% pd_control_qs3_P.HILInitialize_POFrequency
                    section.data(14).logicalSrcIdx = 17;
                    section.data(14).dtTransOffset = 15;

                    ;% pd_control_qs3_P.HILInitialize_POLeading
                    section.data(15).logicalSrcIdx = 18;
                    section.data(15).dtTransOffset = 16;

                    ;% pd_control_qs3_P.HILInitialize_POTrailing
                    section.data(16).logicalSrcIdx = 19;
                    section.data(16).dtTransOffset = 17;

                    ;% pd_control_qs3_P.HILInitialize_POInitial
                    section.data(17).logicalSrcIdx = 20;
                    section.data(17).dtTransOffset = 18;

                    ;% pd_control_qs3_P.HILInitialize_OOInitial
                    section.data(18).logicalSrcIdx = 21;
                    section.data(18).dtTransOffset = 19;

                    ;% pd_control_qs3_P.HILInitialize_OOWatchdog
                    section.data(19).logicalSrcIdx = 22;
                    section.data(19).dtTransOffset = 22;

                    ;% pd_control_qs3_P.SmoothSignalGenerator_InitialPh
                    section.data(20).logicalSrcIdx = 23;
                    section.data(20).dtTransOffset = 25;

                    ;% pd_control_qs3_P.SmoothSignalGenerator_Amplitude
                    section.data(21).logicalSrcIdx = 24;
                    section.data(21).dtTransOffset = 26;

                    ;% pd_control_qs3_P.SmoothSignalGenerator_Frequency
                    section.data(22).logicalSrcIdx = 25;
                    section.data(22).dtTransOffset = 27;

                    ;% pd_control_qs3_P.Amplituderad_Gain
                    section.data(23).logicalSrcIdx = 26;
                    section.data(23).dtTransOffset = 28;

                    ;% pd_control_qs3_P.countstorads_Gain
                    section.data(24).logicalSrcIdx = 27;
                    section.data(24).dtTransOffset = 29;

                    ;% pd_control_qs3_P.countsstorads_Gain
                    section.data(25).logicalSrcIdx = 28;
                    section.data(25).dtTransOffset = 30;

                    ;% pd_control_qs3_P.u0VLimit_UpperSat
                    section.data(26).logicalSrcIdx = 29;
                    section.data(26).dtTransOffset = 31;

                    ;% pd_control_qs3_P.u0VLimit_LowerSat
                    section.data(27).logicalSrcIdx = 30;
                    section.data(27).dtTransOffset = 32;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILInitialize_CKChannels
                    section.data(1).logicalSrcIdx = 31;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.HILInitialize_DOWatchdog
                    section.data(2).logicalSrcIdx = 32;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_P.HILInitialize_EIInitial
                    section.data(3).logicalSrcIdx = 33;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_P.HILInitialize_POModes
                    section.data(4).logicalSrcIdx = 34;
                    section.data(4).dtTransOffset = 3;

                    ;% pd_control_qs3_P.HILInitialize_POConfiguration
                    section.data(5).logicalSrcIdx = 35;
                    section.data(5).dtTransOffset = 4;

                    ;% pd_control_qs3_P.HILInitialize_POAlignment
                    section.data(6).logicalSrcIdx = 36;
                    section.data(6).dtTransOffset = 5;

                    ;% pd_control_qs3_P.HILInitialize_POPolarity
                    section.data(7).logicalSrcIdx = 37;
                    section.data(7).dtTransOffset = 6;

                    ;% pd_control_qs3_P.HILReadTimebase_Clock
                    section.data(8).logicalSrcIdx = 38;
                    section.data(8).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 11;
            section.data(11)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILInitialize_AIChannels
                    section.data(1).logicalSrcIdx = 39;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.HILInitialize_AOChannels
                    section.data(2).logicalSrcIdx = 40;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_P.HILInitialize_DOChannels
                    section.data(3).logicalSrcIdx = 41;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_P.HILInitialize_EIChannels
                    section.data(4).logicalSrcIdx = 42;
                    section.data(4).dtTransOffset = 3;

                    ;% pd_control_qs3_P.HILInitialize_EIQuadrature
                    section.data(5).logicalSrcIdx = 43;
                    section.data(5).dtTransOffset = 5;

                    ;% pd_control_qs3_P.HILInitialize_OOChannels
                    section.data(6).logicalSrcIdx = 44;
                    section.data(6).dtTransOffset = 6;

                    ;% pd_control_qs3_P.HILReadTimebase_SamplesInBuffer
                    section.data(7).logicalSrcIdx = 45;
                    section.data(7).dtTransOffset = 9;

                    ;% pd_control_qs3_P.HILReadTimebase_AnalogChannels
                    section.data(8).logicalSrcIdx = 46;
                    section.data(8).dtTransOffset = 10;

                    ;% pd_control_qs3_P.HILReadTimebase_EncoderChannels
                    section.data(9).logicalSrcIdx = 47;
                    section.data(9).dtTransOffset = 11;

                    ;% pd_control_qs3_P.HILReadTimebase_DigitalChannels
                    section.data(10).logicalSrcIdx = 48;
                    section.data(10).dtTransOffset = 13;

                    ;% pd_control_qs3_P.HILReadTimebase_OtherChannels
                    section.data(11).logicalSrcIdx = 49;
                    section.data(11).dtTransOffset = 14;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 37;
            section.data(37)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILInitialize_Active
                    section.data(1).logicalSrcIdx = 50;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_P.HILInitialize_AOTerminate
                    section.data(2).logicalSrcIdx = 51;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_P.HILInitialize_AOExit
                    section.data(3).logicalSrcIdx = 52;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_P.HILInitialize_DOTerminate
                    section.data(4).logicalSrcIdx = 53;
                    section.data(4).dtTransOffset = 3;

                    ;% pd_control_qs3_P.HILInitialize_DOExit
                    section.data(5).logicalSrcIdx = 54;
                    section.data(5).dtTransOffset = 4;

                    ;% pd_control_qs3_P.HILInitialize_POTerminate
                    section.data(6).logicalSrcIdx = 55;
                    section.data(6).dtTransOffset = 5;

                    ;% pd_control_qs3_P.HILInitialize_POExit
                    section.data(7).logicalSrcIdx = 56;
                    section.data(7).dtTransOffset = 6;

                    ;% pd_control_qs3_P.HILInitialize_CKPStart
                    section.data(8).logicalSrcIdx = 57;
                    section.data(8).dtTransOffset = 7;

                    ;% pd_control_qs3_P.HILInitialize_CKPEnter
                    section.data(9).logicalSrcIdx = 58;
                    section.data(9).dtTransOffset = 8;

                    ;% pd_control_qs3_P.HILInitialize_CKStart
                    section.data(10).logicalSrcIdx = 59;
                    section.data(10).dtTransOffset = 9;

                    ;% pd_control_qs3_P.HILInitialize_CKEnter
                    section.data(11).logicalSrcIdx = 60;
                    section.data(11).dtTransOffset = 10;

                    ;% pd_control_qs3_P.HILInitialize_AIPStart
                    section.data(12).logicalSrcIdx = 61;
                    section.data(12).dtTransOffset = 11;

                    ;% pd_control_qs3_P.HILInitialize_AIPEnter
                    section.data(13).logicalSrcIdx = 62;
                    section.data(13).dtTransOffset = 12;

                    ;% pd_control_qs3_P.HILInitialize_AOPStart
                    section.data(14).logicalSrcIdx = 63;
                    section.data(14).dtTransOffset = 13;

                    ;% pd_control_qs3_P.HILInitialize_AOPEnter
                    section.data(15).logicalSrcIdx = 64;
                    section.data(15).dtTransOffset = 14;

                    ;% pd_control_qs3_P.HILInitialize_AOStart
                    section.data(16).logicalSrcIdx = 65;
                    section.data(16).dtTransOffset = 15;

                    ;% pd_control_qs3_P.HILInitialize_AOEnter
                    section.data(17).logicalSrcIdx = 66;
                    section.data(17).dtTransOffset = 16;

                    ;% pd_control_qs3_P.HILInitialize_AOReset
                    section.data(18).logicalSrcIdx = 67;
                    section.data(18).dtTransOffset = 17;

                    ;% pd_control_qs3_P.HILInitialize_DOPStart
                    section.data(19).logicalSrcIdx = 68;
                    section.data(19).dtTransOffset = 18;

                    ;% pd_control_qs3_P.HILInitialize_DOPEnter
                    section.data(20).logicalSrcIdx = 69;
                    section.data(20).dtTransOffset = 19;

                    ;% pd_control_qs3_P.HILInitialize_DOStart
                    section.data(21).logicalSrcIdx = 70;
                    section.data(21).dtTransOffset = 20;

                    ;% pd_control_qs3_P.HILInitialize_DOEnter
                    section.data(22).logicalSrcIdx = 71;
                    section.data(22).dtTransOffset = 21;

                    ;% pd_control_qs3_P.HILInitialize_DOReset
                    section.data(23).logicalSrcIdx = 72;
                    section.data(23).dtTransOffset = 22;

                    ;% pd_control_qs3_P.HILInitialize_EIPStart
                    section.data(24).logicalSrcIdx = 73;
                    section.data(24).dtTransOffset = 23;

                    ;% pd_control_qs3_P.HILInitialize_EIPEnter
                    section.data(25).logicalSrcIdx = 74;
                    section.data(25).dtTransOffset = 24;

                    ;% pd_control_qs3_P.HILInitialize_EIStart
                    section.data(26).logicalSrcIdx = 75;
                    section.data(26).dtTransOffset = 25;

                    ;% pd_control_qs3_P.HILInitialize_EIEnter
                    section.data(27).logicalSrcIdx = 76;
                    section.data(27).dtTransOffset = 26;

                    ;% pd_control_qs3_P.HILInitialize_POPStart
                    section.data(28).logicalSrcIdx = 77;
                    section.data(28).dtTransOffset = 27;

                    ;% pd_control_qs3_P.HILInitialize_POPEnter
                    section.data(29).logicalSrcIdx = 78;
                    section.data(29).dtTransOffset = 28;

                    ;% pd_control_qs3_P.HILInitialize_POStart
                    section.data(30).logicalSrcIdx = 79;
                    section.data(30).dtTransOffset = 29;

                    ;% pd_control_qs3_P.HILInitialize_POEnter
                    section.data(31).logicalSrcIdx = 80;
                    section.data(31).dtTransOffset = 30;

                    ;% pd_control_qs3_P.HILInitialize_POReset
                    section.data(32).logicalSrcIdx = 81;
                    section.data(32).dtTransOffset = 31;

                    ;% pd_control_qs3_P.HILInitialize_OOReset
                    section.data(33).logicalSrcIdx = 82;
                    section.data(33).dtTransOffset = 32;

                    ;% pd_control_qs3_P.HILInitialize_DOFinal
                    section.data(34).logicalSrcIdx = 83;
                    section.data(34).dtTransOffset = 33;

                    ;% pd_control_qs3_P.HILInitialize_DOInitial
                    section.data(35).logicalSrcIdx = 84;
                    section.data(35).dtTransOffset = 34;

                    ;% pd_control_qs3_P.HILReadTimebase_Active
                    section.data(36).logicalSrcIdx = 85;
                    section.data(36).dtTransOffset = 35;

                    ;% pd_control_qs3_P.HILWrite_Active
                    section.data(37).logicalSrcIdx = 86;
                    section.data(37).dtTransOffset = 36;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_P.HILReadTimebase_OverflowMode
                    section.data(1).logicalSrcIdx = 87;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 2;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (pd_control_qs3_B)
        ;%
            section.nData     = 14;
            section.data(14)  = dumData; %prealloc

                    ;% pd_control_qs3_B.HILReadTimebase_o1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_B.HILReadTimebase_o2
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_B.HILReadTimebase_o3
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_B.HILReadTimebase_o5
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% pd_control_qs3_B.HILReadTimebase_o6
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% pd_control_qs3_B.SmoothSignalGenerator
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% pd_control_qs3_B.Amplituderad
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% pd_control_qs3_B.countstorads
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% pd_control_qs3_B.Sum
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% pd_control_qs3_B.SliderGain
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% pd_control_qs3_B.countsstorads
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% pd_control_qs3_B.SliderGain_b
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% pd_control_qs3_B.Sum1
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% pd_control_qs3_B.u0VLimit
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_B.HILReadTimebase_o4
                    section.data(1).logicalSrcIdx = 14;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 2;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (pd_control_qs3_DW)
        ;%
            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILInitialize_FilterFrequency
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_DW.HILReadTimebase_AnalogBuffer
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 2;

                    ;% pd_control_qs3_DW.HILReadTimebase_OtherBuffer
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 3;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILInitialize_Card
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILReadTimebase_Task
                    section.data(1).logicalSrcIdx = 4;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILWrite_PWORK
                    section.data(1).logicalSrcIdx = 6;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_DW.Positionrad_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 7;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_DW.VmV_PWORK.LoggedData
                    section.data(3).logicalSrcIdx = 8;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILInitialize_ClockModes
                    section.data(1).logicalSrcIdx = 9;
                    section.data(1).dtTransOffset = 0;

                    ;% pd_control_qs3_DW.HILInitialize_DOStates
                    section.data(2).logicalSrcIdx = 10;
                    section.data(2).dtTransOffset = 1;

                    ;% pd_control_qs3_DW.HILInitialize_QuadratureModes
                    section.data(3).logicalSrcIdx = 11;
                    section.data(3).dtTransOffset = 2;

                    ;% pd_control_qs3_DW.HILInitialize_InitialEICounts
                    section.data(4).logicalSrcIdx = 12;
                    section.data(4).dtTransOffset = 4;

                    ;% pd_control_qs3_DW.HILReadTimebase_EncoderBuffer
                    section.data(5).logicalSrcIdx = 13;
                    section.data(5).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% pd_control_qs3_DW.HILReadTimebase_DigitalBuffer
                    section.data(1).logicalSrcIdx = 14;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 2627873013;
    targMap.checksum1 = 1361881096;
    targMap.checksum2 = 2052606178;
    targMap.checksum3 = 3885486571;

