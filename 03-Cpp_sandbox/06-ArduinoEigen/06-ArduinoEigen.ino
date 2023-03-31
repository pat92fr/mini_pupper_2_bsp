//https://github.com/bolderflight/Eigen
#include "eigen.h"

  float const STANCE_X {0.0f};
  float const STANCE_Y {0.0f};
  float const STANCE_Z {0.0f};
  float const STANCE_X_SHIFT {0.0f};

        Eigen::Matrix<float,3,4> standby_pose_BRF {
            {  STANCE_X+STANCE_X_SHIFT, STANCE_X+STANCE_X_SHIFT,-STANCE_X+STANCE_X_SHIFT,  -STANCE_X+STANCE_X_SHIFT },
            { -STANCE_Y,                STANCE_Y,               -STANCE_Y,                  STANCE_Y },
            {  STANCE_Z,                STANCE_Z,                STANCE_Z,                  STANCE_Z }
        };

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
