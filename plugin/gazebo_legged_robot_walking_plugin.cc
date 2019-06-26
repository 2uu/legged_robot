#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>

#define LINK_NUM 3  // 全リンク数 (total number of links)
#define JT_NUM   3  // 全ジョイント数 (total number of joints)                                           
#define LEG_NUM  4  // 全脚数 (total number of legs) 

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
public:
  MobileBasePlugin()
  {
    l1 = 1.5*0.05, l2 = 0.3, l3  = 0.3;  // leg length
    r1 = 0.02, r2 = 0.02, r3 = 0.02 ;    // leg radius
    
    for (int i=0; i< 4; i++) {
      for (int j=0; j < 3; j++) { 	 				
         THETA[i][j] = 0; 			
      } 		
    } 	
   } 	

   void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 	
   { 		
      physics::WorldPtr world = physics::get_world("default"); // modelへポインタを格納
      this->model = _parent;
    
      hinge_lf1 = this->model->GetJoint("hinge_lf1");
      hinge_lf2 = this->model->GetJoint("hinge_lf2");
      hinge_lf3 = this->model->GetJoint("hinge_lf3");
      hinge_lr1 = this->model->GetJoint("hinge_lr1");
      hinge_lr2 = this->model->GetJoint("hinge_lr2");
      hinge_lr3 = this->model->GetJoint("hinge_lr3");
      hinge_rr1 = this->model->GetJoint("hinge_rr1");
      hinge_rr2 = this->model->GetJoint("hinge_rr2");
      hinge_rr3 = this->model->GetJoint("hinge_rr3");
      hinge_rf1 = this->model->GetJoint("hinge_rf1");
      hinge_rf2 = this->model->GetJoint("hinge_rf2");
      hinge_rf3 = this->model->GetJoint("hinge_rf3");
       
    // このプラグイン用のパラメータをロード
    if (this->LoadParams(_sdf)) {
      // アップテートイベントを聞く。シミュレーションの繰り返し時に
      // このイベントはブロードキャストされる。
      this->updateConnection
	       = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&MobileBasePlugin::OnUpdate, this));
    }
  }
  
  bool LoadParams(sdf::ElementPtr _sdf)
  {
    // 制御用のgainパラメータを見つける
    if (!_sdf->HasElement("gain")) {
      gzerr << "param [gain] not found\n"; 
      return false;
    }     
    else {   			// gainの値を取得
      this->gain = _sdf->Get<double>("gain");
    }
    
    // 成功時
    return true;
  }
  
  /*** 逆運動学の計算 (calculate inverse kinematics ***/
  void  inverseKinematics(double x, double y, double z,
    double *ang1, double *ang2, double *ang3,int posture)
  {
    double l1a = 0, l3a = l3 + r3/2;
    double c3 = (x*x + z*z + (y-l1a)*(y-l1a) - (l2*l2+l3a*l3a))/(2*l2*l3a);
    double s2 = (y-l1a) / (l2 + l3a*c3);
    double c2 = sqrt(1 - s2 * s2);
    double c1 = (l2 + l3a*c3)*c2/sqrt(x*x+z*z);
    // printf("c3=%f s2=%f c2=%f c1=%f \n", c3,s2,c2,c1);
    if (sqrt(x*x+y*y+z*z) > l2 + l3) {
      printf(" Target point is out of range \n");
    }
    
    switch (posture) {
    case 1: // 姿勢１ (posture 1)
      *ang1 =   atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1);
      *ang2 = - atan2(s2,c2);
      *ang3 =   atan2(sqrt(1-c3*c3),c3); break;
    case 2: // 姿勢２ (posture 2)
      *ang1=   atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
      *ang2= - atan2(s2,c2);
      *ang3= - atan2(sqrt(1-c3*c3),c3); break;
    case 3:  // 姿勢３ (posture 3)
      *ang1 =   M_PI + (atan2(x,-z) - atan2(sqrt(1 - c1*c1),c1));
      *ang2 = - M_PI +  atan2(s2,c2);
      *ang3 = - atan2(sqrt(1-c3*c3),c3); break;
    case 4:  // 姿勢４ (posture 4)
      *ang1 =  M_PI + atan2(x,-z) + atan2(sqrt(1 - c1*c1),c1);
      *ang2 = -M_PI + atan2(s2,c2);
      *ang3 =  atan2(sqrt(1-c3*c3),c3); break;
    }
  }


  /*** 関節を動かす ***/
  void moveJoint()
  {
    hinge_lf1->SetPosition(0, THETA[0][0]);
    hinge_lr1->SetPosition(0, THETA[1][0]);
    hinge_rr1->SetPosition(0, THETA[2][0]);
    hinge_rf1->SetPosition(0, THETA[3][0]);
    hinge_lf2->SetPosition(0, THETA[0][1]);
    hinge_lr2->SetPosition(0, THETA[1][1]);
    hinge_rr2->SetPosition(0, THETA[2][1]);
    hinge_rf2->SetPosition(0, THETA[3][1]);
    hinge_lf3->SetPosition(0, THETA[0][2]);
    hinge_lr3->SetPosition(0, THETA[1][2]);
    hinge_rr3->SetPosition(0, THETA[2][2]);
    hinge_rf3->SetPosition(0, THETA[3][2]);
  }

  /*** 歩行制御 (gait control) ***/
  void walk()
  {
    static int t = 0, steps = 0;
    int interval = 1000;
    
    if ((steps++ % interval)==0){
      t++;
    }
    else {  // 目標関節角度の設定 (set target gait angles)
      for (int leg_no = 0; leg_no < LEG_NUM; leg_no++) {
	for (int joint_no = 0; joint_no < JT_NUM; joint_no++) {
	  THETA[leg_no][joint_no] = gait[t%12][leg_no][joint_no];
	}
      }
    }
    moveJoint();
  }


  void calcAngle()  /*** 目標角度の計算 (Calculate target angles) ***/
  {
    double z0 = -0.4,z1 = -0.37; // z0:地面までの高さ(height to the ground)，z1:最高到達点 (highest point)
    double y1 = 0.05, fs = 0.2;  // y1:左右の変位(defference between right and left)，fs:歩幅 (foot step)
    double f1 = fs/4, f2 = fs/2, f3 = 3 * fs/4, f4 = fs;  // 一時変数 (temporal variables)
    double  traj[12][LEG_NUM][3] = { // 目標軌道点 (trajectory points)
      // leg0: left fore leg,  leg1: left rear leg
      // leg2: right rear leg, leg3: right fore leg
      // 左前leg0遊脚 左後leg1    右後 leg2    右前leg3
      {{ 0, y1,z0},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 0 重心移動(move COG)
      {{f2, y1,z1},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 1 離地(takeoff)
      {{f4, y1,z0},{  0, y1,z0},{  0, y1,z0},{  0, y1,z0}},// 2 着地(touchdown)
      {{f3,-y1,z0},{-f1,-y1,z0},{-f1,-y1,z0},{-f1,-y1,z0}},// 3 重心移動(move COG)
      //    leg0        leg1     leg2 遊脚(swing)  leg3
      {{f3,-y1,z0},{-f1,-y1,z0},{ f1,-y1,z1},{-f1,-y1,z0}},// 4 離地(takeoff)
      {{f3,-y1,z0},{-f1,-y1,z0},{ f3,-y1,z0},{-f1,-y1,z0}},// 5 着地(touchdown)
      {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{-f2,-y1,z0}},// 6 重心移動(move COG)
      //    leg0        leg1      leg2       leg3 遊脚(swing)
      {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{  0,-y1,z1}},// 7 離地(takeoff)
      {{f2,-y1,z0},{-f2,-y1,z0},{ f2,-y1,z0},{ f2,-y1,z0}},// 8 着地(touchdown)
      {{f1, y1,z0},{-f3, y1,z0},{ f1, y1,z0},{ f1, y1,z0}},// 9 重心移動(move COG)
      //   leg0    leg1 遊脚(swing)  leg2         leg3
      {{f1,y1, z0},{-f1, y1,z1},{ f1, y1,z0},{ f1, y1,z0}},// 10 離地(takeoff)
      {{f1,y1, z0},{ f1, y1,z0},{ f1, y1,z0},{ f1, y1,z0}} // 11 着地(touchdown)
    };
    
    double angle1, angle2, angle3;
    int posture = 2; // 姿勢(posture)
    for (int i = 0; i < 12; i++) {
      for (int k = 0; k < LEG_NUM; k++) {
	// 逆運動学
        inverseKinematics(traj[i][k][0],traj[i][k][1], 
	traj[i][k][2],&angle1, &angle2, &angle3,posture);
	gait[i][k][0] = angle1;
	gait[i][k][1] = angle2;
	gait[i][k][2] = angle3;
      }
    }
  }

  // ワールド更新開始イベントから呼び出される
  void OnUpdate()
  {
    static int epoch = 0;
    //std::cout << "epoch=" << epoch << std::endl;
    if (epoch++ == 0) calcAngle();
    walk();
  }

private:
  // モデルへのポインタ
  physics::ModelPtr model;
  physics::WorldPtr world;

  // ワールド状態のサブスクライブ(講読)
  transport::NodePtr node;
  transport::SubscriberPtr statsSub;
  common::Time simTime;

  // 更新イベントコネクションへのポインタ
  event::ConnectionPtr updateConnection;
  physics::JointPtr hinge_lf1, hinge_lf2, hinge_lf3; // 左前脚の関節
  physics::JointPtr hinge_lr1, hinge_lr2, hinge_lr3; // 左後
  physics::JointPtr hinge_rr1, hinge_rr2, hinge_rr3; // 右後
  physics::JointPtr hinge_rf1, hinge_rf2, hinge_rf3; // 右前
  //sensors::RaySensorPtr laser;
  //physics::LinkPtr  sensor;
  double THETA[LEG_NUM][LINK_NUM];    // 関節目標角度 
  double gait[12][LEG_NUM][JT_NUM] ;  // 目標角度(target angle of gait)                           
  double l1, l2, l3;  // 脚長 (lenth of links)   
  double r1, r2, r3;  // 脚半径(leg radius)
  double gain;     // 比例ゲイン
};

// シミュレータへのプラグイン登録
GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)

}
