#include "servo_kinematics.h"
#include "servo.h"
#include "farm_scene_config.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Complete prefab transform chain; no planar/coaxial approximation.
 * Bounded damped least squares, fixed-size storage, no heap allocation.
 * mode 0: position with fixed S5; 1: position + tool yaw; 2: full pose.
 */
static const double offsets[5][3] = {
    ARM_SHOULDER_OFFSET, ARM_ELBOW_OFFSET, ARM_WRIST_OFFSET,
    ARM_ROLL_OFFSET, ARM_TOOL_OFFSET
};
static const double signs[5] = {
    S1_SERVO_SIGN, S2_SERVO_SIGN, S3_SERVO_SIGN, S4_SERVO_SIGN, S5_SERVO_SIGN
};
static const double zeros[5] = {
    S1_ZERO_OFFSET, S2_ZERO_OFFSET, S3_ZERO_OFFSET, S4_ZERO_OFFSET, S5_ZERO_OFFSET
};
static const double joint_min[5] = {J1_MIN,J2_MIN,J3_MIN,J4_MIN,J5_MIN};
static const double joint_max[5] = {J1_MAX,J2_MAX,J3_MAX,J4_MAX,J5_MAX};

static double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int valid_angles(const double a[5])
{
    for (int i=0;i<5;i++) {
        double q=(a[i]*RAD2DEG-zeros[i])/signs[i];
        if (!isfinite(a[i]) || fabs(a[i]) > PI/2+1e-10 ||
            q < joint_min[i]-1e-8 || q > joint_max[i]+1e-8) return 0;
    }
    return 1;
}

static double bounded_angle(int i,double a)
{
    double lo=(zeros[i]+signs[i]*joint_min[i])*DEG2RAD;
    double hi=(zeros[i]+signs[i]*joint_max[i])*DEG2RAD;
    if (lo>hi) { double t=lo;lo=hi;hi=t; }
    return clamp(a,fmax(lo,-PI/2),fmin(hi,PI/2));
}

void Matrix4x4_Mult(const double A[4][4],const double B[4][4],double C[4][4])
{
    double tmp[4][4]={{0}};
    for(int i=0;i<4;i++) for(int j=0;j<4;j++)
        for(int k=0;k<4;k++) tmp[i][j]+=A[i][k]*B[k][j];
    memcpy(C,tmp,sizeof(tmp));
}

void Matrix4x4_VecMult(const double T[4][4],const double v[4],double out[4])
{
    double tmp[4]={0};
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) tmp[i]+=T[i][j]*v[j];
    memcpy(out,tmp,sizeof(tmp));
}

/* Postmultiply by an elementary rotation. Unity Euler(x,y,z) = Ry*Rx*Rz. */
static void rotate(double R[3][3],int axis,double a)
{
    int u=(axis+1)%3,v=(axis+2)%3;
    double c=cos(a),s=sin(a);
    for(int i=0;i<3;i++) {
        double x=R[i][u],y=R[i][v];
        R[i][u]=c*x+s*y; R[i][v]=-s*x+c*y;
    }
}

void Kinematics_Forward(const double theta[5],double T[4][4])
{
    double q[5],R[3][3]={{1,0,0},{0,1,0},{0,0,1}},p[3]={0};
    const int row[3]={2,0,1}; /* Unity (right,up,forward) -> (forward,right,up) */
    for(int i=0;i<5;i++) q[i]=(theta[i]-zeros[i]*DEG2RAD)/signs[i];
    rotate(R,1,-q[0]);
    for(int link=0;link<5;link++) {
        for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            p[i]+=R[i][j]*offsets[link][j];
        if(link==0) { rotate(R,1,PI); rotate(R,0,q[1]); }
        if(link==1) { rotate(R,0,q[2]+PI/2); rotate(R,2,PI/2); }
        if(link==2) { rotate(R,1,-q[3]-PI/2); rotate(R,2,PI/2); }
        if(link==3) rotate(R,1,-q[4]);
    }
    for(int i=0;i<3;i++) {
        /* Tool Z is the finger approach axis (Unity local Y). */
        T[i][0]=R[row[i]][0]; T[i][1]=-R[row[i]][2];
        T[i][2]=R[row[i]][1]; T[i][3]=p[row[i]];
        T[3][i]=0;
    }
    T[3][3]=1;
}

/* Rotation entries avoid Euler singularities and angle wrapping. */
static int residual(const double a[5],const double target[4][4],int mode,
                    double yaw,double e[12],double *score)
{
    double T[4][4];
    int n=3;
    Kinematics_Forward(a,T);
    for(int i=0;i<3;i++) e[i]=target[i][3]-T[i][3];
    if(mode==1) {
        double h=hypot(T[0][0],T[1][0]);
        e[n++]=.03*(cos(yaw)-T[0][0]/fmax(h,1e-12));
        e[n++]=.03*(sin(yaw)-T[1][0]/fmax(h,1e-12));
    } else if(mode==2) {
        for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            e[n++]=.05*(target[i][j]-T[i][j]);
    }
    *score=0;
    for(int i=0;i<n;i++) *score+=e[i]*e[i];
    return n;
}

static int reached(const double a[5],const double target[4][4],int mode,double yaw)
{
    double T[4][4],d2=0;
    if(!valid_angles(a)) return 0;
    Kinematics_Forward(a,T);
    for(int i=0;i<3;i++) { double d=T[i][3]-target[i][3];d2+=d*d; }
    if(d2>ARM_IK_POSITION_TOL_M*ARM_IK_POSITION_TOL_M) return 0;
    if(mode==1) {
        double h=hypot(T[0][0],T[1][0]);
        if(h<1e-8 || (T[0][0]*cos(yaw)+T[1][0]*sin(yaw))/h<cos(.2*DEG2RAD))
            return 0;
    }
    if(mode==2) for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        if(fabs(T[i][j]-target[i][j])>ARM_IK_ORIENTATION_TOL) return 0;
    return 1;
}

/* Pivoted Gaussian elimination of a damped 5x5 system. */
static int linear_solve(double A[5][6],int n,double x[5])
{
    for(int k=0;k<n;k++) {
        int pivot=k;
        for(int i=k+1;i<n;i++) if(fabs(A[i][k])>fabs(A[pivot][k])) pivot=i;
        if(fabs(A[pivot][k])<1e-16) return 0;
        for(int j=k;j<=n;j++) { double t=A[k][j];A[k][j]=A[pivot][j];A[pivot][j]=t; }
        double div=A[k][k];
        for(int j=k;j<=n;j++) A[k][j]/=div;
        for(int i=0;i<n;i++) if(i!=k) {
            double f=A[i][k];
            for(int j=k;j<=n;j++) A[i][j]-=f*A[k][j];
        }
    }
    for(int i=0;i<n;i++) x[i]=A[i][n];
    return 1;
}

static int refine(double a[5],const double target[4][4],int mode,double yaw)
{
    const int nv=mode==0?4:5;
    double damping=1e-5;
    for(int iter=0;iter<100;iter++) {
        double e[12],score,J[12][5],A[5][6]={{0}},delta[5]={0};
        int n=residual(a,target,mode,yaw,e,&score);
        if(reached(a,target,mode,yaw)) return 1;
        for(int j=0;j<nv;j++) {
            double b[5],ep[12],sp;
            memcpy(b,a,sizeof(b)); b[j]+=1e-5;
            residual(b,target,mode,yaw,ep,&sp);
            for(int i=0;i<n;i++) J[i][j]=(ep[i]-e[i])/1e-5;
        }
        for(int j=0;j<nv;j++) {
            for(int k=0;k<nv;k++) for(int i=0;i<n;i++) A[j][k]+=J[i][j]*J[i][k];
            A[j][j]+=damping;
            for(int i=0;i<n;i++) A[j][nv]-=J[i][j]*e[i];
        }
        if(!linear_solve(A,nv,delta)) return 0;
        double max_step=0;
        for(int j=0;j<nv;j++) max_step=fmax(max_step,fabs(delta[j]));
        double scale=max_step>.25?.25/max_step:1;
        int accepted=0;
        for(int ls=0;ls<7;ls++,scale*=.5) {
            double b[5],ep[12],sp;
            memcpy(b,a,sizeof(b));
            for(int j=0;j<nv;j++) b[j]=bounded_angle(j,a[j]+scale*delta[j]);
            residual(b,target,mode,yaw,ep,&sp);
            if(sp<score-1e-16) {
                memcpy(a,b,sizeof(b));accepted=1;damping=fmax(1e-8,damping*.5);break;
            }
        }
        if(!accepted) { damping*=10;if(damping>.1) break; }
    }
    return reached(a,target,mode,yaw);
}

static int valid_target(const double T[4][4],int mode)
{
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) if(!isfinite(T[i][j])) return 0;
    if(hypot(hypot(T[0][3],T[1][3]),T[2][3])>.20) return 0;
    if(mode!=2) return 1;
    for(int i=0;i<4;i++) if(fabs(T[3][i]-(i==3?1:0))>1e-6) return 0;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
        double dot=0;for(int k=0;k<3;k++) dot+=T[k][i]*T[k][j];
        if(fabs(dot-(i==j?1:0))>1e-5) return 0;
    }
    double det=T[0][0]*(T[1][1]*T[2][2]-T[1][2]*T[2][1])
              -T[0][1]*(T[1][0]*T[2][2]-T[1][2]*T[2][0])
              +T[0][2]*(T[1][0]*T[2][1]-T[1][1]*T[2][0]);
    return fabs(det-1)<1e-5;
}

static int inverse(const double T[4][4],int mode,double yaw,
                   const double current[5],double sols[4][5])
{
    static const double seeds[][3] = {
        {30,50,30},{50,30,50},{60,-20,60},{10,70,10},
        {-30,-50,-30},{-50,-30,-50},{-60,20,-60},{-10,-70,-10},
        {0,0,0},{70,70,-50},{-70,-70,50},{45,-60,-45}
    };
    int count=0;
    if(!valid_target(T,mode) || !isfinite(yaw) || !valid_angles(current)) return 0;
    for(int seed=-1;seed<(int)(sizeof(seeds)/sizeof(seeds[0]));seed++) {
        double a[5];
        memcpy(a,current,sizeof(a));
        if(seed>=0) {
            /* Positive shoulder bends towards -forward at base zero. */
            double base=atan2(T[1][3],-T[0][3]);
            if(seed>=4 && seed<8) base=atan2(-T[1][3],T[0][3]);
            a[0]=bounded_angle(0,base);
            for(int j=1;j<4;j++) a[j]=bounded_angle(j,seeds[seed][j-1]*DEG2RAD);
            if(mode!=0) a[4]=bounded_angle(4,((seed%3)-1)*PI/3);
        }
        if(!refine(a,T,mode,yaw)) continue;
        int duplicate=0;
        for(int i=0;i<count;i++) {
            double d=0;for(int j=0;j<5;j++) d+=fabs(sols[i][j]-a[j]);
            if(d<.01) duplicate=1;
        }
        if(!duplicate) { memcpy(sols[count++],a,sizeof(a));if(count==4) break; }
    }
    return count;
}

int Kinematics_InversePositionOnly(double x,double y,double z,double yaw,double sols[4][5])
{
    double T[4][4]={{1,0,0,x},{0,1,0,y},{0,0,1,z},{0,0,0,1}},current[5]={0};
    return inverse(T,1,yaw,current,sols);
}

int Kinematics_PositionFixedWrist(double x,double y,double z,
                                  const double current[5],double sols[4][5])
{
    double T[4][4]={{1,0,0,x},{0,1,0,y},{0,0,1,z},{0,0,0,1}};
    return inverse(T,0,0,current,sols);
}

int Kinematics_Inverse(const double T[4][4],double sols[4][5])
{
    double current[5]={0};
    return inverse(T,2,0,current,sols);
}

void Kinematics_SelectOptimal(const double current[5],const double sols[4][5],
                              int num,double optimal[5])
{
    double best=1e100;
    for(int i=0;i<num && i<4;i++) {
        if(!valid_angles(sols[i])) continue;
        double cost=0;
        for(int j=0;j<5;j++) {
            /* Bounded servos cannot take a wrapped path through +/-180. */
            double d=sols[i][j]-current[j];cost+=d*d;
        }
        if(cost<best) { best=cost;memcpy(optimal,sols[i],5*sizeof(double)); }
    }
}

static int plan_position(double x,double y,double z,const double seed[5],int16_t target[6])
{
    double T[4][4]={{1,0,0,x},{0,1,0,y},{0,0,1,z},{0,0,0,1}};
    double sol[4][5],best[5],quantized[5],actual[4][4],error2=0;
    int n=inverse(T,0,0,seed,sol);
    if(!n) return -1; /* 上层统一打印失败原因，避免重复日志。 */
    /* First solution follows the calibrated approach posture, not the folded
     * home pose. Nearest-angle selection alone can choose a colliding branch. */
    memcpy(best,sol[0],sizeof(best));
    if(!valid_angles(best)) return -1;
    for(int i=0;i<5;i++) {
        double tenth=best[i]*RAD2DEG*10;
        target[i]=(int16_t)(tenth>=0?tenth+.5:tenth-.5);
        if(target[i]<-900 || target[i]>900) return -1;
        quantized[i]=target[i]*DEG2RAD/10;
    }
    Kinematics_Forward(quantized,actual);
    for(int i=0;i<3;i++) { double d=actual[i][3]-T[i][3];error2+=d*d; }
    if(error2>ARM_COMMAND_POSITION_TOL_M*ARM_COMMAND_POSITION_TOL_M) return -1;
    return 0;
}

int move_to_position(double x,double y,double z)
{
    int16_t target[6],drop[6];
    double seed[5]={87.5*DEG2RAD,40*DEG2RAD,40*DEG2RAD,40*DEG2RAD,0};
    double drop_seed[5]={0};
    double radial=hypot(x,y);
    if(!isfinite(radial)||radial<FARM_ARM_RADIAL_MIN_M||radial>FARM_ARM_RADIAL_MAX_M) return -1;
    for(int i=0;i<5;i++) if(current_angles[i]<-900||current_angles[i]>900)return -1;
    seed[4]=current_angles[4]*DEG2RAD/10;
    if(plan_position(x,y,z,seed,target)<0 ||
       plan_position(FARM_BIN_DROP_X_M,FARM_BIN_DROP_Y_M,FARM_BIN_DROP_Z_M,drop_seed,drop)<0)
        return -1; /* Validate pick AND release before issuing any motion. */
    int16_t upright[6]={current_angles[0],0,0,0,current_angles[4],GRIPPER_OPEN};
    move_smooth(upright,STEP_NUM);
    upright[0]=target[0];
    move_smooth(upright,STEP_NUM); /* Rotate only with the arm raised. */
    target[5]=GRIPPER_OPEN;
    move_smooth(target,STEP_NUM);
    SMART_DELAY_MS(400);
    set_gripper_servo(GRIPPER_CLOSE);
    SMART_DELAY_MS(1200);
    return 0;
}

int release_gripper(void)
{
    double seed[5]={0};
    int16_t target[6];
    if(plan_position(FARM_BIN_DROP_X_M,FARM_BIN_DROP_Y_M,FARM_BIN_DROP_Z_M,seed,target)<0)
        return -1;
    int16_t upright[6]={current_angles[0],0,0,0,current_angles[4],GRIPPER_CLOSE};
    move_smooth(upright,STEP_NUM); /* Keep holding while lifting out of the plant. */
    upright[0]=target[0];upright[4]=0;
    move_smooth(upright,STEP_NUM);
    target[5]=GRIPPER_CLOSE;
    move_smooth(target,STEP_NUM); /* Bin opening, not the old side-release pose. */
    SMART_DELAY_MS(400);
    set_gripper_servo(GRIPPER_OPEN);
    SMART_DELAY_MS(2000);
    printf("[采摘] 松爪等待结束，开始抬臂回位\n");
    upright[5]=GRIPPER_OPEN;
    move_smooth(upright,STEP_NUM);
    upright[0]=0;
    move_smooth(upright,STEP_NUM);
    return 0;
}

void servo_test(void)
{
    int16_t home[6]={0,0,0,0,0,GRIPPER_OPEN};
    SERVO_S1_Init();SERVO_S2_Init();SERVO_S3456_Init();
    move_smooth(home,STEP_NUM);
    SMART_DELAY_MS(1000);
    if(move_to_position(FARM_SENSOR_FORWARD_OFFSET_M,FARM_SENSOR_RIGHT_OFFSET_M+
        FARM_NOMINAL_PEPPER_DISTANCE_MM/1000.0+FARM_SURFACE_TO_GRIP_M,
        FARM_SENSOR_HEIGHT_OFFSET_M+FARM_GRIP_ABOVE_SENSOR_M)==0) {
        SMART_DELAY_MS(1000);release_gripper();
    }
    while(1) {}
}
