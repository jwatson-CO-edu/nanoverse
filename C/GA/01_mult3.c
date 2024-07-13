typedef struct{
    double ps;
    double e1;
    double e2;
    double e3;
    double e12;
    double e13;
    double e23;
    double e123;
}mv3;

mv3 wedge_mv3( mv3 op1, mv3 op2 ){
   mv3 wdgProd = {
        // 0-blade //
        0.0, // op1.ps * op2.ps
        // 1-blades //
        0.0, // op1.ps * op2.e1 + op2.ps * op1.e1,
        0.0, // op1.ps * op2.e2 + op2.ps * op1.e2,
        0.0, // op1.ps * op2.e3 + op2.ps * op1.e3,
        // 2-blades //
        op1.e1 * op2.e2 + op2.e1 * op1.e2,
        op1.e1 * op2.e3 + op2.e1 * op1.e3,
        op1.e2 * op2.e3 + op2.e2 * op1.e3,
        // 3-blade //
        op1.e1 * op2.e23 + op2.e1 * op1.e23  +  op1.e2 * op2.e13 + op2.e2 * op1.e13  +  op1.e3 * op2.e12 + op2.e3 * op1.e12
            + op1.e123 * op2.e123
   };
   return wdgProd;
}