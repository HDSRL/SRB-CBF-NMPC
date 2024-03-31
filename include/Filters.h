#ifndef FILTERS_H
#define FILTERS_H

#include "stdlib.h"

typedef struct FilterStructure_d {
    double** rawHist;
    double** filtHist;
    double* a;
    double* b;
    int numCol;
    int numRow;
}FiltStruct_d;

typedef struct FilterStructure_f {
    float** rawHist;
    float** filtHist;
    float* a;
    float* b;
    int numCol;
    int numRow;
}FiltStruct_f;

/*
This filter assumes that the newest data points are always in the
first row of the array, and different data channels are in columns.

a and b are assumed to be the same length. If using populate_filter,
the coefficients do not need to be expressed in the form of a 
proper transfer function (the conversion is taken care of). 
However, it is important to note that discrete_butter does require
a proper transfer function. If not using populare_filter, be sure
to use proper coefficients accordingly.

@params
FiltStruct_d* obj: filter object (ideally created using populate_filter_d)
double* newData: most recent raw reading to be filtered
*/
inline void discrete_butter_d(FiltStruct_d* obj, double* newData){
    for(int i=0; i<obj->numCol; ++i){
        for(int j=obj->numRow-1; j>0; --j){
            obj->filtHist[j][i] = obj->filtHist[j-1][i];
            obj->rawHist[j][i] = obj->rawHist[j-1][i];
        }
        obj->rawHist[0][i] = newData[i];
        obj->filtHist[0][i] = 0;
    }
    for(int i=0; i<obj->numCol; ++i){
        for(int j=0; j<obj->numRow; ++j){
            obj->filtHist[0][i] += (obj->b[j]*obj->rawHist[j][i]-obj->a[j]*obj->filtHist[j][i]);
        }
        newData[i] = obj->filtHist[0][i];
    }
}

/*
This populates FiltStruct_d. Note: a and b are assumed to be the same length.

@params
FiltStruct_d* obj: the structure to be populated
const double* a: the denominator of the filter transfer function
const double* b: the numerator of the filter transfer function
int numRow: the filter order +1
int numCol: the number of channels to filter (same as the number of sensor readings to be filtered)
*/
inline void populate_filter_d(FiltStruct_d* obj, const double* a, const double* b , int numRow, int numCol){
    obj->rawHist  = (double**)malloc(numRow*sizeof(double*));
    obj->filtHist = (double**)malloc(numRow*sizeof(double*));
    obj->a = (double*)malloc(sizeof(double)*numRow);
    obj->b = (double*)malloc(sizeof(double)*numRow);
    for(int i=0; i<numRow; ++i){
        obj->rawHist[i]  = (double*)malloc(numCol*sizeof(double*));
        obj->filtHist[i] = (double*)malloc(numCol*sizeof(double*));
        for(int j=0; j<numCol; ++j){
            obj->rawHist[i][j] = 0.0;
            obj->filtHist[i][j] = 0.0;
        }
    }
    for(int i=0; i<numRow; ++i){
        obj->a[i] = a[i]/a[0];
        obj->b[i] = b[i]/a[0];
    }
    obj->numCol = numCol;
    obj->numRow = numRow;
}

/*
This function frees all of the memory in FiltStruct_d
*/
inline void clear_filter_d(FiltStruct_d* obj){
    for(int i=0; i<obj->numRow; ++i){
        free(obj->rawHist[i]);
        free(obj->filtHist[i]);
    }
    free(obj->rawHist);
    free(obj->filtHist);
    free(obj->a);
    free(obj->b);
}

inline void discrete_butter_f(FiltStruct_f* obj, float* newData){
    for(int i=0; i<obj->numCol; ++i){
        for(int j=obj->numRow-1; j>0; --j){
            obj->filtHist[j][i] = obj->filtHist[j-1][i];
            obj->rawHist[j][i] = obj->rawHist[j-1][i];
        }
        obj->rawHist[0][i] = newData[i];
        obj->filtHist[0][i] = 0;
    }
    for(int i=0; i<obj->numCol; ++i){
        for(int j=0; j<obj->numRow; ++j){
            obj->filtHist[0][i] += (obj->b[j]*obj->rawHist[j][i]-obj->a[j]*obj->filtHist[j][i]);
        }
        newData[i] = obj->filtHist[0][i];
    }
}

inline void populate_filter_f(FiltStruct_f* obj, const float* a, const float* b , int numRow, int numCol){
    obj->rawHist  = (float**)malloc(numRow*sizeof(float*));
    obj->filtHist = (float**)malloc(numRow*sizeof(float*));
    obj->a = (float*)malloc(sizeof(float)*numRow);
    obj->b = (float*)malloc(sizeof(float)*numRow);
    for(int i=0; i<numRow; ++i){
        obj->rawHist[i]  = (float*)malloc(numCol*sizeof(float*));
        obj->filtHist[i] = (float*)malloc(numCol*sizeof(float*));
        for(int j=0; j<numCol; ++j){
            obj->rawHist[i][j] = 0.0;
            obj->filtHist[i][j] = 0.0;
        }
    }
    for(int i=0; i<numRow; ++i){
        obj->a[i] = a[i]/a[0];
        obj->b[i] = b[i]/a[0];
    }
    obj->numRow = numRow;
    obj->numCol = numCol;
}

inline void clear_filter_f(FiltStruct_f* obj){
    for(int i=0; i<obj->numRow; ++i){
        free(obj->rawHist[i]);
        free(obj->filtHist[i]);
    }
    free(obj->rawHist);
    free(obj->filtHist);
    free(obj->a);
    free(obj->b);
    free(obj);
}

#endif