#include "memory"
#include "iostream"
#include "stdint.h"


class bar{
public:
    bar(){std::cout<<"made bar"<<std::endl;};
    virtual ~bar(){std::cout<<"destroyed bar"<<std::endl;};
    int X = 1;
};


class foo : public bar{
public:    
    foo(){
       std::cout << "made foo" <<std::endl;
    };
    virtual ~foo(){std::cout<<"destroyed foo"<<std::endl;};

};



int main(){	
	uint16_t X=15;
	uint16_t X_temp = ~X;
	std::cout<<X_temp<<std::endl;
	

    return 0;
}

