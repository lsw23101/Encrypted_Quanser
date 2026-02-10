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
        ;% Auto data (q_qube2_swingup_P)
        ;%
            section.nData     = 11;
            section.data(11)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.Jp
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_P.Lp
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_P.Lr
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_P.Mp
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_P.Mr
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% q_qube2_swingup_P.Rm
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% q_qube2_swingup_P.g
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% q_qube2_swingup_P.kt
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% q_qube2_swingup_P.EnableBalanceControl_const
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% q_qube2_swingup_P.mu_gain
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% q_qube2_swingup_P.Er_gain
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.HILWriteAnalog_channels
                    section.data(1).logicalSrcIdx = 11;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 42;
            section.data(42)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.uKx_Gain
                    section.data(1).logicalSrcIdx = 12;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_P.HILInitialize_OOTerminate
                    section.data(2).logicalSrcIdx = 13;
                    section.data(2).dtTransOffset = 4;

                    ;% q_qube2_swingup_P.HILInitialize_OOExit
                    section.data(3).logicalSrcIdx = 14;
                    section.data(3).dtTransOffset = 5;

                    ;% q_qube2_swingup_P.HILInitialize_OOStart
                    section.data(4).logicalSrcIdx = 15;
                    section.data(4).dtTransOffset = 6;

                    ;% q_qube2_swingup_P.HILInitialize_OOEnter
                    section.data(5).logicalSrcIdx = 16;
                    section.data(5).dtTransOffset = 7;

                    ;% q_qube2_swingup_P.HILInitialize_AOFinal
                    section.data(6).logicalSrcIdx = 17;
                    section.data(6).dtTransOffset = 8;

                    ;% q_qube2_swingup_P.HILInitialize_POFinal
                    section.data(7).logicalSrcIdx = 18;
                    section.data(7).dtTransOffset = 9;

                    ;% q_qube2_swingup_P.HILInitialize_OOFinal
                    section.data(8).logicalSrcIdx = 19;
                    section.data(8).dtTransOffset = 10;

                    ;% q_qube2_swingup_P.HILInitialize_AIHigh
                    section.data(9).logicalSrcIdx = 20;
                    section.data(9).dtTransOffset = 13;

                    ;% q_qube2_swingup_P.HILInitialize_AILow
                    section.data(10).logicalSrcIdx = 21;
                    section.data(10).dtTransOffset = 14;

                    ;% q_qube2_swingup_P.HILInitialize_AOHigh
                    section.data(11).logicalSrcIdx = 22;
                    section.data(11).dtTransOffset = 15;

                    ;% q_qube2_swingup_P.HILInitialize_AOLow
                    section.data(12).logicalSrcIdx = 23;
                    section.data(12).dtTransOffset = 16;

                    ;% q_qube2_swingup_P.HILInitialize_AOInitial
                    section.data(13).logicalSrcIdx = 24;
                    section.data(13).dtTransOffset = 17;

                    ;% q_qube2_swingup_P.HILInitialize_AOWatchdog
                    section.data(14).logicalSrcIdx = 25;
                    section.data(14).dtTransOffset = 18;

                    ;% q_qube2_swingup_P.HILInitialize_POFrequency
                    section.data(15).logicalSrcIdx = 26;
                    section.data(15).dtTransOffset = 19;

                    ;% q_qube2_swingup_P.HILInitialize_POLeading
                    section.data(16).logicalSrcIdx = 27;
                    section.data(16).dtTransOffset = 20;

                    ;% q_qube2_swingup_P.HILInitialize_POTrailing
                    section.data(17).logicalSrcIdx = 28;
                    section.data(17).dtTransOffset = 21;

                    ;% q_qube2_swingup_P.HILInitialize_POInitial
                    section.data(18).logicalSrcIdx = 29;
                    section.data(18).dtTransOffset = 22;

                    ;% q_qube2_swingup_P.HILInitialize_OOInitial
                    section.data(19).logicalSrcIdx = 30;
                    section.data(19).dtTransOffset = 23;

                    ;% q_qube2_swingup_P.HILInitialize_OOWatchdog
                    section.data(20).logicalSrcIdx = 31;
                    section.data(20).dtTransOffset = 26;

                    ;% q_qube2_swingup_P.Pendulumcountstorad_Gain
                    section.data(21).logicalSrcIdx = 32;
                    section.data(21).dtTransOffset = 29;

                    ;% q_qube2_swingup_P.Constant1_Value
                    section.data(22).logicalSrcIdx = 33;
                    section.data(22).dtTransOffset = 30;

                    ;% q_qube2_swingup_P.Bias_Bias
                    section.data(23).logicalSrcIdx = 34;
                    section.data(23).dtTransOffset = 31;

                    ;% q_qube2_swingup_P.u_max_Value
                    section.data(24).logicalSrcIdx = 35;
                    section.data(24).dtTransOffset = 32;

                    ;% q_qube2_swingup_P.Constant_Value
                    section.data(25).logicalSrcIdx = 36;
                    section.data(25).dtTransOffset = 33;

                    ;% q_qube2_swingup_P.Constant_Value_k
                    section.data(26).logicalSrcIdx = 37;
                    section.data(26).dtTransOffset = 34;

                    ;% q_qube2_swingup_P.alpha_dot_A
                    section.data(27).logicalSrcIdx = 38;
                    section.data(27).dtTransOffset = 35;

                    ;% q_qube2_swingup_P.alpha_dot_C
                    section.data(28).logicalSrcIdx = 39;
                    section.data(28).dtTransOffset = 36;

                    ;% q_qube2_swingup_P.alpha_dot_D
                    section.data(29).logicalSrcIdx = 40;
                    section.data(29).dtTransOffset = 37;

                    ;% q_qube2_swingup_P.mJtoJ_Gain
                    section.data(30).logicalSrcIdx = 41;
                    section.data(30).dtTransOffset = 38;

                    ;% q_qube2_swingup_P.SetpointStateXd_Value
                    section.data(31).logicalSrcIdx = 42;
                    section.data(31).dtTransOffset = 39;

                    ;% q_qube2_swingup_P.Armcountstorad_Gain
                    section.data(32).logicalSrcIdx = 43;
                    section.data(32).dtTransOffset = 43;

                    ;% q_qube2_swingup_P.theta_dot_A
                    section.data(33).logicalSrcIdx = 44;
                    section.data(33).dtTransOffset = 44;

                    ;% q_qube2_swingup_P.theta_dot_C
                    section.data(34).logicalSrcIdx = 45;
                    section.data(34).dtTransOffset = 45;

                    ;% q_qube2_swingup_P.theta_dot_D
                    section.data(35).logicalSrcIdx = 46;
                    section.data(35).dtTransOffset = 46;

                    ;% q_qube2_swingup_P.alpha_dot_A_o
                    section.data(36).logicalSrcIdx = 47;
                    section.data(36).dtTransOffset = 47;

                    ;% q_qube2_swingup_P.alpha_dot_C_i
                    section.data(37).logicalSrcIdx = 48;
                    section.data(37).dtTransOffset = 48;

                    ;% q_qube2_swingup_P.alpha_dot_D_i
                    section.data(38).logicalSrcIdx = 49;
                    section.data(38).dtTransOffset = 49;

                    ;% q_qube2_swingup_P.ForveCCW_Gain
                    section.data(39).logicalSrcIdx = 50;
                    section.data(39).dtTransOffset = 50;

                    ;% q_qube2_swingup_P.Gain_Gain
                    section.data(40).logicalSrcIdx = 51;
                    section.data(40).dtTransOffset = 51;

                    ;% q_qube2_swingup_P.JtomJ_Gain
                    section.data(41).logicalSrcIdx = 52;
                    section.data(41).dtTransOffset = 52;

                    ;% q_qube2_swingup_P.Gain_Gain_a
                    section.data(42).logicalSrcIdx = 53;
                    section.data(42).dtTransOffset = 53;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.HILInitialize_CKChannels
                    section.data(1).logicalSrcIdx = 54;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_P.HILInitialize_DOWatchdog
                    section.data(2).logicalSrcIdx = 55;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_P.HILInitialize_EIInitial
                    section.data(3).logicalSrcIdx = 56;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_P.HILInitialize_POModes
                    section.data(4).logicalSrcIdx = 57;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_P.HILInitialize_POConfiguration
                    section.data(5).logicalSrcIdx = 58;
                    section.data(5).dtTransOffset = 4;

                    ;% q_qube2_swingup_P.HILInitialize_POAlignment
                    section.data(6).logicalSrcIdx = 59;
                    section.data(6).dtTransOffset = 5;

                    ;% q_qube2_swingup_P.HILInitialize_POPolarity
                    section.data(7).logicalSrcIdx = 60;
                    section.data(7).dtTransOffset = 6;

                    ;% q_qube2_swingup_P.HILReadEncoderTimebase_Clock
                    section.data(8).logicalSrcIdx = 61;
                    section.data(8).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 8;
            section.data(8)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.HILInitialize_AIChannels
                    section.data(1).logicalSrcIdx = 62;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_P.HILInitialize_AOChannels
                    section.data(2).logicalSrcIdx = 63;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_P.HILInitialize_DOChannels
                    section.data(3).logicalSrcIdx = 64;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_P.HILInitialize_EIChannels
                    section.data(4).logicalSrcIdx = 65;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_P.HILInitialize_EIQuadrature
                    section.data(5).logicalSrcIdx = 66;
                    section.data(5).dtTransOffset = 5;

                    ;% q_qube2_swingup_P.HILInitialize_OOChannels
                    section.data(6).logicalSrcIdx = 67;
                    section.data(6).dtTransOffset = 6;

                    ;% q_qube2_swingup_P.HILReadEncoderTimebase_Channels
                    section.data(7).logicalSrcIdx = 68;
                    section.data(7).dtTransOffset = 9;

                    ;% q_qube2_swingup_P.HILReadEncoderTimebase_SamplesI
                    section.data(8).logicalSrcIdx = 69;
                    section.data(8).dtTransOffset = 11;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 37;
            section.data(37)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.HILInitialize_Active
                    section.data(1).logicalSrcIdx = 70;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_P.HILInitialize_AOTerminate
                    section.data(2).logicalSrcIdx = 71;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_P.HILInitialize_AOExit
                    section.data(3).logicalSrcIdx = 72;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_P.HILInitialize_DOTerminate
                    section.data(4).logicalSrcIdx = 73;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_P.HILInitialize_DOExit
                    section.data(5).logicalSrcIdx = 74;
                    section.data(5).dtTransOffset = 4;

                    ;% q_qube2_swingup_P.HILInitialize_POTerminate
                    section.data(6).logicalSrcIdx = 75;
                    section.data(6).dtTransOffset = 5;

                    ;% q_qube2_swingup_P.HILInitialize_POExit
                    section.data(7).logicalSrcIdx = 76;
                    section.data(7).dtTransOffset = 6;

                    ;% q_qube2_swingup_P.HILInitialize_CKPStart
                    section.data(8).logicalSrcIdx = 77;
                    section.data(8).dtTransOffset = 7;

                    ;% q_qube2_swingup_P.HILInitialize_CKPEnter
                    section.data(9).logicalSrcIdx = 78;
                    section.data(9).dtTransOffset = 8;

                    ;% q_qube2_swingup_P.HILInitialize_CKStart
                    section.data(10).logicalSrcIdx = 79;
                    section.data(10).dtTransOffset = 9;

                    ;% q_qube2_swingup_P.HILInitialize_CKEnter
                    section.data(11).logicalSrcIdx = 80;
                    section.data(11).dtTransOffset = 10;

                    ;% q_qube2_swingup_P.HILInitialize_AIPStart
                    section.data(12).logicalSrcIdx = 81;
                    section.data(12).dtTransOffset = 11;

                    ;% q_qube2_swingup_P.HILInitialize_AIPEnter
                    section.data(13).logicalSrcIdx = 82;
                    section.data(13).dtTransOffset = 12;

                    ;% q_qube2_swingup_P.HILInitialize_AOPStart
                    section.data(14).logicalSrcIdx = 83;
                    section.data(14).dtTransOffset = 13;

                    ;% q_qube2_swingup_P.HILInitialize_AOPEnter
                    section.data(15).logicalSrcIdx = 84;
                    section.data(15).dtTransOffset = 14;

                    ;% q_qube2_swingup_P.HILInitialize_AOStart
                    section.data(16).logicalSrcIdx = 85;
                    section.data(16).dtTransOffset = 15;

                    ;% q_qube2_swingup_P.HILInitialize_AOEnter
                    section.data(17).logicalSrcIdx = 86;
                    section.data(17).dtTransOffset = 16;

                    ;% q_qube2_swingup_P.HILInitialize_AOReset
                    section.data(18).logicalSrcIdx = 87;
                    section.data(18).dtTransOffset = 17;

                    ;% q_qube2_swingup_P.HILInitialize_DOPStart
                    section.data(19).logicalSrcIdx = 88;
                    section.data(19).dtTransOffset = 18;

                    ;% q_qube2_swingup_P.HILInitialize_DOPEnter
                    section.data(20).logicalSrcIdx = 89;
                    section.data(20).dtTransOffset = 19;

                    ;% q_qube2_swingup_P.HILInitialize_DOStart
                    section.data(21).logicalSrcIdx = 90;
                    section.data(21).dtTransOffset = 20;

                    ;% q_qube2_swingup_P.HILInitialize_DOEnter
                    section.data(22).logicalSrcIdx = 91;
                    section.data(22).dtTransOffset = 21;

                    ;% q_qube2_swingup_P.HILInitialize_DOReset
                    section.data(23).logicalSrcIdx = 92;
                    section.data(23).dtTransOffset = 22;

                    ;% q_qube2_swingup_P.HILInitialize_EIPStart
                    section.data(24).logicalSrcIdx = 93;
                    section.data(24).dtTransOffset = 23;

                    ;% q_qube2_swingup_P.HILInitialize_EIPEnter
                    section.data(25).logicalSrcIdx = 94;
                    section.data(25).dtTransOffset = 24;

                    ;% q_qube2_swingup_P.HILInitialize_EIStart
                    section.data(26).logicalSrcIdx = 95;
                    section.data(26).dtTransOffset = 25;

                    ;% q_qube2_swingup_P.HILInitialize_EIEnter
                    section.data(27).logicalSrcIdx = 96;
                    section.data(27).dtTransOffset = 26;

                    ;% q_qube2_swingup_P.HILInitialize_POPStart
                    section.data(28).logicalSrcIdx = 97;
                    section.data(28).dtTransOffset = 27;

                    ;% q_qube2_swingup_P.HILInitialize_POPEnter
                    section.data(29).logicalSrcIdx = 98;
                    section.data(29).dtTransOffset = 28;

                    ;% q_qube2_swingup_P.HILInitialize_POStart
                    section.data(30).logicalSrcIdx = 99;
                    section.data(30).dtTransOffset = 29;

                    ;% q_qube2_swingup_P.HILInitialize_POEnter
                    section.data(31).logicalSrcIdx = 100;
                    section.data(31).dtTransOffset = 30;

                    ;% q_qube2_swingup_P.HILInitialize_POReset
                    section.data(32).logicalSrcIdx = 101;
                    section.data(32).dtTransOffset = 31;

                    ;% q_qube2_swingup_P.HILInitialize_OOReset
                    section.data(33).logicalSrcIdx = 102;
                    section.data(33).dtTransOffset = 32;

                    ;% q_qube2_swingup_P.HILInitialize_DOFinal
                    section.data(34).logicalSrcIdx = 103;
                    section.data(34).dtTransOffset = 33;

                    ;% q_qube2_swingup_P.HILInitialize_DOInitial
                    section.data(35).logicalSrcIdx = 104;
                    section.data(35).dtTransOffset = 34;

                    ;% q_qube2_swingup_P.HILReadEncoderTimebase_Active
                    section.data(36).logicalSrcIdx = 105;
                    section.data(36).dtTransOffset = 35;

                    ;% q_qube2_swingup_P.HILWriteAnalog_Active
                    section.data(37).logicalSrcIdx = 106;
                    section.data(37).dtTransOffset = 36;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_P.HILReadEncoderTimebase_Overflow
                    section.data(1).logicalSrcIdx = 107;
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
        nTotSects     = 3;
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
        ;% Auto data (q_qube2_swingup_B)
        ;%
            section.nData     = 37;
            section.data(37)  = dumData; %prealloc

                    ;% q_qube2_swingup_B.HILReadEncoderTimebase_o1
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_B.HILReadEncoderTimebase_o2
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_B.Pendulumcountstorad
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_B.MathFunction
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_B.Bias
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 4;

                    ;% q_qube2_swingup_B.alpha
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 5;

                    ;% q_qube2_swingup_B.SliderGain
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 6;

                    ;% q_qube2_swingup_B.cosalpha
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 7;

                    ;% q_qube2_swingup_B.Sum1
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 8;

                    ;% q_qube2_swingup_B.PendTorqueNm
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 9;

                    ;% q_qube2_swingup_B.alpha_dot
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 10;

                    ;% q_qube2_swingup_B.alpha_dot2
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 11;

                    ;% q_qube2_swingup_B.PendInertiakgm2
                    section.data(13).logicalSrcIdx = 12;
                    section.data(13).dtTransOffset = 12;

                    ;% q_qube2_swingup_B.Energy
                    section.data(14).logicalSrcIdx = 13;
                    section.data(14).dtTransOffset = 13;

                    ;% q_qube2_swingup_B.SliderGain_j
                    section.data(15).logicalSrcIdx = 14;
                    section.data(15).dtTransOffset = 14;

                    ;% q_qube2_swingup_B.mJtoJ
                    section.data(16).logicalSrcIdx = 15;
                    section.data(16).dtTransOffset = 15;

                    ;% q_qube2_swingup_B.cosalpha_g
                    section.data(17).logicalSrcIdx = 16;
                    section.data(17).dtTransOffset = 16;

                    ;% q_qube2_swingup_B.UnaryMinus
                    section.data(18).logicalSrcIdx = 17;
                    section.data(18).dtTransOffset = 17;

                    ;% q_qube2_swingup_B.Armcountstorad
                    section.data(19).logicalSrcIdx = 18;
                    section.data(19).dtTransOffset = 18;

                    ;% q_qube2_swingup_B.theta_dot
                    section.data(20).logicalSrcIdx = 19;
                    section.data(20).dtTransOffset = 19;

                    ;% q_qube2_swingup_B.alpha_dot_b
                    section.data(21).logicalSrcIdx = 20;
                    section.data(21).dtTransOffset = 20;

                    ;% q_qube2_swingup_B.EnableBalanceControlSwitch
                    section.data(22).logicalSrcIdx = 21;
                    section.data(22).dtTransOffset = 21;

                    ;% q_qube2_swingup_B.ForveCCW
                    section.data(23).logicalSrcIdx = 22;
                    section.data(23).dtTransOffset = 22;

                    ;% q_qube2_swingup_B.Gain
                    section.data(24).logicalSrcIdx = 23;
                    section.data(24).dtTransOffset = 23;

                    ;% q_qube2_swingup_B.JtomJ
                    section.data(25).logicalSrcIdx = 24;
                    section.data(25).dtTransOffset = 24;

                    ;% q_qube2_swingup_B.Gain_b
                    section.data(26).logicalSrcIdx = 25;
                    section.data(26).dtTransOffset = 25;

                    ;% q_qube2_swingup_B.Sum
                    section.data(27).logicalSrcIdx = 26;
                    section.data(27).dtTransOffset = 26;

                    ;% q_qube2_swingup_B.uKx
                    section.data(28).logicalSrcIdx = 27;
                    section.data(28).dtTransOffset = 30;

                    ;% q_qube2_swingup_B.alpha_dotcosalpha
                    section.data(29).logicalSrcIdx = 28;
                    section.data(29).dtTransOffset = 31;

                    ;% q_qube2_swingup_B.Sign
                    section.data(30).logicalSrcIdx = 29;
                    section.data(30).dtTransOffset = 32;

                    ;% q_qube2_swingup_B.EEr
                    section.data(31).logicalSrcIdx = 30;
                    section.data(31).dtTransOffset = 33;

                    ;% q_qube2_swingup_B.EErsigna_dotcosa
                    section.data(32).logicalSrcIdx = 31;
                    section.data(32).dtTransOffset = 34;

                    ;% q_qube2_swingup_B.Product
                    section.data(33).logicalSrcIdx = 32;
                    section.data(33).dtTransOffset = 35;

                    ;% q_qube2_swingup_B.Switch2
                    section.data(34).logicalSrcIdx = 33;
                    section.data(34).dtTransOffset = 36;

                    ;% q_qube2_swingup_B.AccelerationtoTorque
                    section.data(35).logicalSrcIdx = 34;
                    section.data(35).dtTransOffset = 37;

                    ;% q_qube2_swingup_B.TorquetoVoltage
                    section.data(36).logicalSrcIdx = 35;
                    section.data(36).dtTransOffset = 38;

                    ;% q_qube2_swingup_B.Switch
                    section.data(37).logicalSrcIdx = 36;
                    section.data(37).dtTransOffset = 39;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_B.Compare
                    section.data(1).logicalSrcIdx = 37;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% q_qube2_swingup_B.LowerRelop1
                    section.data(1).logicalSrcIdx = 38;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_B.UpperRelop
                    section.data(2).logicalSrcIdx = 39;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
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
        nTotSects     = 5;
        sectIdxOffset = 3;

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
        ;% Auto data (q_qube2_swingup_DW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_DW.HILInitialize_FilterFrequency
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_DW.HILInitialize_Card
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% q_qube2_swingup_DW.HILReadEncoderTimebase_Task
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% q_qube2_swingup_DW.HILWriteAnalog_PWORK
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_DW.MotorVoltageV_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 4;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_DW.Pendulumdeg_PWORK.LoggedData
                    section.data(3).logicalSrcIdx = 5;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_DW.PendulumEnergymJ_PWORK.LoggedData
                    section.data(4).logicalSrcIdx = 6;
                    section.data(4).dtTransOffset = 3;

                    ;% q_qube2_swingup_DW.RotaryArmdeg_PWORK.LoggedData
                    section.data(5).logicalSrcIdx = 7;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% q_qube2_swingup_DW.HILInitialize_ClockModes
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

                    ;% q_qube2_swingup_DW.HILInitialize_DOStates
                    section.data(2).logicalSrcIdx = 9;
                    section.data(2).dtTransOffset = 1;

                    ;% q_qube2_swingup_DW.HILInitialize_QuadratureModes
                    section.data(3).logicalSrcIdx = 10;
                    section.data(3).dtTransOffset = 2;

                    ;% q_qube2_swingup_DW.HILInitialize_InitialEICounts
                    section.data(4).logicalSrcIdx = 11;
                    section.data(4).dtTransOffset = 4;

                    ;% q_qube2_swingup_DW.HILReadEncoderTimebase_Buffer
                    section.data(5).logicalSrcIdx = 12;
                    section.data(5).dtTransOffset = 6;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
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


    targMap.checksum0 = 3609380520;
    targMap.checksum1 = 2756618951;
    targMap.checksum2 = 3410229693;
    targMap.checksum3 = 438695713;

