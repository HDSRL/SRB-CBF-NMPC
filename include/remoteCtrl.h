#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

struct buttons {
    int R1          = 0;
    int L1          = 0;
    int start       = 0;
    int select      = 0;
    int R2          = 0;
    int L2          = 0;
    int F1          = 0;
    int F2          = 0;
    int A           = 0;
    int B           = 0;
    int X           = 0;
    int Y           = 0;
    int up          = 0;
    int right       = 0;
    int down        = 0;
    int left        = 0;
    int32_t RX      = 0;
    int32_t RY      = 0;
    int32_t LX      = 0;
    int32_t LY      = 0;
};


buttons getButtonState(uint8_t remote[40],buttons btnOld){
	buttons btn;

	uint8_t btnset = remote[2];
	int result;
	
	// Buttonset 1
	for (int c = 5; c >= 0; c--){
		result = btnset >> c;
		if(result & 1){
			if(c==0){
				btn.R1 = btnOld.R1>=1 ? 2 : 1;
			}else if(c==1){
				btn.L1 = btnOld.L1>=1 ? 2 : 1;
			}else if(c==2){
				btn.start = btnOld.start>=1 ? 2 : 1;
			}else if(c==3){
				btn.select = btnOld.select>=1 ? 2 : 1;
			}else if(c==4){
				btn.R2 = btnOld.R2>=1 ? 2 : 1;
			}else if(c==5){
				btn.L2 = btnOld.L2>=1 ? 2 : 1;
			}
		}				  
	}
	
	// Buttonset 2
	btnset = remote[3];
	for (int c = 7; c >= 0; c--){
		result = btnset >> c;
		if(result & 1){
			if(c==0){
				btn.A = btnOld.A>=1 ? 2 : 1;
			}else if(c==1){
				btn.B = btnOld.B>=1 ? 2 : 1;
			}else if(c==2){
				btn.X = btnOld.X>=1 ? 2 : 1;
			}else if(c==3){
				btn.Y = btnOld.Y>=1 ? 2 : 1;
			}else if(c==4){
				btn.up = btnOld.up>=1 ? 2 : 1;
			}else if(c==5){
				btn.right = btnOld.right>=1 ? 2 : 1;
			}else if(c==6){
				btn.down = btnOld.down>=1 ? 2 : 1;
			}else if(c==7){
				btn.left = btnOld.left>=1 ? 2 : 1;
			}
		}				  
	}
	return btn;

}

#endif