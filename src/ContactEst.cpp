#include "ContactEst.hpp"

ContactEst::ContactEst(){
    for(int i=0; i<CON_MIN_TIME; ++i){
        histMask <<= 1;
        histMask++;
    }
    for(int i=0; i<NUM_EE; ++i){
        EEMask <<= 1;
        EEMask++;
        hist[i] = histMask;
    }
    est = EEMask;
    rise = histMask;
    stance = histMask;
    
    thresh = 8000;
}

void ContactEst::updateConState(float footPos[4], float phase, const int force[4]){
    // Update the estimator (right now just uses force and a threshold)
    int conInd[4] = {0}; // holds results of kalman
    updateConEst(conInd, footPos, phase, force);

    // Update history
    for (int i=0; i<NUM_EE; ++i){
        hist[i] <<= 1;
        hist[i] += conInd[i];
    }

    // Update est and ctrl
    con_uint numCon = 0;
    for (int i=0; i<NUM_EE; ++i){
        numCon = histMask & hist[i];
        if (numCon==histMask){
            SETBIT(est,BIT_END-i);
        }else if (numCon==0){
            CLEARBIT(est,BIT_END-i);
        }
    }
    con_uint phaseMask = 0;
    if(phase>0.7){
    	phaseMask = EEMask;
    }
    
    stance |= (ctrl & rise & EEMask);
    rise |= ((~est) & EEMask);
    ctrl = (rise & est & phaseMask & EEMask);
    ctrl |= stance;

    con.cnt = 0;
    for(int i=0; i<NUM_EE; ++i){
        con.ind_prev[i] = con.ind[i];
        con.ind[i] = TESTBIT(ctrl,BIT_END-i);
        con.act[i] = TESTBIT(est,BIT_END-i);
        con.cnt += TESTBIT(ctrl,BIT_END-i);
    }
//    if(ctrl>0){
//    	std::cout << "Est:          " << est  << std::endl;
//    	std::cout << "~Est:         " << ((~est)&EEMask)  << std::endl;
//    	std::cout << "Rise:         " << rise << std::endl;
//    	std::cout << "Stance:       " << stance << std::endl;
//    	std::cout << "Control Bits: " << ctrl << std::endl << std::endl;
//    }

    con.changeDomain = ctrl==EEMask;
}

void ContactEst::updateConEst(int conInd[4], float footPos[4], float phase, const int force[4]){
    // Run probability kalman
    // temporarily in place of prob kalman:
    for (int i=0; i<4; ++i){
        conInd[i] = force[i]>thresh;
    }
}

void ContactEst::setDesDomain(std::vector<int> des){
    con.changeDomain = 0;
    rise = 0;
    con.cnt = 0;
    for(int i=0; i<NUM_EE; ++i){
        rise <<= 1;
        rise += des[i];
        con.des[i] = des[i];
        con.ind[i] = des[i];
        con.cnt += des[i];
    }
    stance = rise;
}

void ContactEst::forceDomChange(){
    con.changeDomain = 1;
}

