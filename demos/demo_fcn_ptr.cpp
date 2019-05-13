/*
 * test_fcn_ptr.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: jsola
 */

#include <iostream>
#include <cstdarg>

// define some fcns with differnt # of args
double half     (double _a)                         { return _a/2;      }
double quarter  (double _a)                         { return _a/4;      }
double divide   (double _n, double _d)              { return _n/_d;     }
double mult     (double _x, double _y)              { return _x*_y;     }
double mult_div (double _x, double _y, double _z)   { return _x*_y/_z;  }

//======== test_simple usage of function pointers ============
typedef double (*FcnType)(double);

double run_simple(FcnType f, double a){ return f(a); }

void test_simple()
{
    std::cout << "simple..." << std::endl;
    std::cout << "4/2   = " << run_simple(half,    4) << std::endl;
    std::cout << "4/4   = " << run_simple(quarter, 4) << std::endl;
}

//======== more usage of function pointers ============
typedef double (*FcnType2)(double a, double b);
typedef double (*FcnType3)(double a, double b, double c);

double run_2(FcnType2 f, double a, double b) { return f(a, b); }
double run_3(FcnType3 f, double a, double b, double c) { return f(a, b, c); }

void test_more()
{
    std::cout << "more..." << std::endl;
    std::cout << "4/2   = " << run_2(divide,   4, 2)    << std::endl;
    std::cout << "4*2   = " << run_2(mult,     4, 2)    << std::endl;
    std::cout << "4*3/6 = " << run_3(mult_div, 4, 3, 6) << std::endl;
}

//======== variadic usage of function pointers =========
typedef double (*FcnTypeV)(...);

// we will try to use half(), quarter(), mult(), divide() and mult_div() above

//------------------------------------------------------------------------------------
// ---- try just to read the args of the variadic fcn; no function pointer yet
double run_v_dummy(int n, ...)
{
    va_list args;
    va_start(args, n);
    double b = 0;
    for (int i=0; i<n; i++)
        b += va_arg(args, double);
    return b;
}

void test_var_dummy()
{
    std::cout << "var dummy..." << std::endl;
    std::cout << "1     = " << run_v_dummy(1, 1.0)           << std::endl;
    std::cout << "1+2   = " << run_v_dummy(2, 1.0, 2.0)      << std::endl;
    std::cout << "1+2+3 = " << run_v_dummy(3, 1.0, 2.0, 3.0) << std::endl;
}

//------------------------------------------------------------------------------------
// ---- call function through pointer; ugly solution with switch / case on the # of args:
double run_v_switch(FcnTypeV f, int n, ...)
{
    va_list args ;
    va_start(args, n); // args start after n
    switch (n)
    {
        case 1:
            return f(va_arg(args, double));
            break;
        case 2:
            return f(va_arg(args, double), va_arg(args, double));
            break;
        case 3:
            return f(va_arg(args, double), va_arg(args, double), va_arg(args, double));
            break;
        default:
            return 0;
    }
}

void test_var_switch()
{
    std::cout << "var using switch/case of the # of args (ugly)..." << std::endl;
    std::cout << "4/2   = " << run_v_switch((FcnTypeV)half,     1, 4.0          ) << std::endl;
    std::cout << "4/4   = " << run_v_switch((FcnTypeV)quarter,  1, 4.0          ) << std::endl;
    std::cout << "4/2   = " << run_v_switch((FcnTypeV)divide,   2, 4.0, 2.0     ) << std::endl;
    std::cout << "4*2   = " << run_v_switch((FcnTypeV)mult,     2, 4.0, 2.0     ) << std::endl;
    std::cout << "4*3/6 = " << run_v_switch((FcnTypeV)mult_div, 3, 4.0, 3.0, 6.0) << std::endl;
}

//------------------------------------------------------------------------------------
// ---- call function through pointer; try to forward all args straight into the inner function!
double run_v(FcnTypeV f, int n, ...)
{
    va_list args;
    va_start(args,n);   // args start after n
    return f(args);     // hop!
}

void test_var()
{
    std::cout << "var forwarding all args to the inner fcn (nice!)..." << std::endl;
    std::cout << "4/2   = " << run_v((FcnTypeV)half,     1, 4.0          ) << std::endl;
    std::cout << "4/4   = " << run_v((FcnTypeV)quarter,  1, 4.0          ) << std::endl;
    std::cout << "4/2   = " << run_v((FcnTypeV)divide,   2, 4.0, 2.0     ) << std::endl;
    std::cout << "4*2   = " << run_v((FcnTypeV)mult,     2, 4.0, 2.0     ) << std::endl;
    std::cout << "4*3/6 = " << run_v((FcnTypeV)mult_div, 3, 4.0, 3.0, 6.0) << std::endl;
}

//####################################################################################

int main()
{
    test_simple();
    test_more();
    test_var_dummy();
    test_var_switch();
    test_var();
}
