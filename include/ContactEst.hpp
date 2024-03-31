#ifndef CONTACT_ESTIMATION
#define CONTACT_ESTIMATION

#include "global_loco_structs.hpp"
#include "stdint.h"

#define NUM_EE 4 // DO NOT CHANGE CURRENTLY
#define CON_MIN_TIME 10 // Must be >0 and <=64. Time required before change is considered 

#if (CON_MIN_TIME <= 8)
    typedef uint16_t con_uint;
#elif (CON_MIN_TIME <= 16)
    typedef uint16_t con_uint;
#elif (CON_MIN_TIME <= 32)
    typedef uint32_t con_uint;
#else
    typedef uint64_t con_uint;
#endif
#define BIT_END NUM_EE-1

#define SETBIT(X,n)       ( (X) |=  ( (con_uint)1 << (n) ) )
#define CLEARBIT(X,n)     ( (X) &= ~( (con_uint)1 << (n) ) )
#define TESTBIT(X,n)    ( ( (X) &   ( (con_uint)1 << (n) ) ) > 0 )


class ContactEst{
    public:
        ContactEst();
        virtual ~ContactEst(){};

        void updateConState(float footPos[4], float phase, const int force[4]);
        void setDesDomain(std::vector<int> des);
        void forceDomChange();
        const ContactInfo* getConInfoPointer(){return &con;};

    private:
        void updateConEst(int conInd[4], float footPos[4], float phase, const int force[4]);

        float thresh = 50;

        con_uint histMask = 0;
        con_uint EEMask = 0;
        con_uint rise = 0;
        con_uint stance = 0;
        con_uint hist[NUM_EE] = {0};
        con_uint est = 0;
        con_uint ctrl = 0;

        ContactInfo con;
};


#endif
