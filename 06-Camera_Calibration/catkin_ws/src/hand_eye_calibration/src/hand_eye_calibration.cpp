#include <eigen3/Eigen/Dense> 
#include <fstream> 
#include <iomanip>
#include <cstdio> 
#include <iostream> 

class HandEyeCalibration
{
    public:
        HandEyeCalibration(char* );
        bool ParseCalibrationFile(char* );
        void LeastSquareAlgorithm();

    private:
        std::ifstream Calibration_txt;
        std::string line;
        Eigen::MatrixXf ArmPosition;
        Eigen::MatrixXf EyePosition;
        Eigen::Vector3f ArmCentroid;
        Eigen::Vector3f EyeCentroid;
        Eigen::Matrix3f Rotation;
        Eigen::Array3f Translation;
        Eigen::Matrix4f Homogeneous;
};

bool HandEyeCalibration::ParseCalibrationFile(char* file)
{

    int count = 0;
    Calibration_txt.open(file, std::ifstream::in);
            
    if(!Calibration_txt)
    {
        printf("Can not open the calibration.txt!\n");
        printf("Check the address is correct!\n");
        return false;
    }
            
    while(std::getline(Calibration_txt, line)) 
    {
        if(!line.empty()) 
            ++count;
    }
    ArmPosition = Eigen::MatrixXf(count, 3);
    EyePosition = Eigen::MatrixXf(count, 3);
            

    Calibration_txt.clear();
    Calibration_txt.seekg(0, std::ios::beg);

    int row = 0;
    while(!Calibration_txt.eof() && row<count)
    {
        std::getline(Calibration_txt, line);
        std::stringstream str(line);
        int i = 0;
        while(str.good() && i<6)
        {
            double val;
            switch(i)
            {
                case 0: case 1: case 2:
                    str >> val; 
                    ArmPosition(row, i) = val; 
                    break;
                case 3: case 4: case 5:
                    str >> val; 
                    EyePosition(row, i-3) = val; 
                    break;
            } 
            ++i;
        } 
        ++row;
    }
    return true;
}

void HandEyeCalibration::LeastSquareAlgorithm()
{
    double arm_sum = 0.0, eye_sum = 0.0;
    for(int i=0; i<3; ++i)
    {
        for(int j=0; j<ArmPosition.rows(); ++j)
        {
            arm_sum += ArmPosition(j, i);
            eye_sum += EyePosition(j, i);
        }
        ArmCentroid(i) = arm_sum/(double)ArmPosition.rows(); 
        EyeCentroid(i) = eye_sum/(double)EyePosition.rows(); 
        arm_sum = eye_sum = 0;
    }


    Eigen::MatrixXf ArmDeviation(ArmPosition.rows(), 3);
    Eigen::MatrixXf EyeDeviation(EyePosition.rows(), 3);
    for(int i=0; i<ArmPosition.rows(); ++i)
    {
        for(int j=0; j<3; ++j)
        {
            ArmDeviation(i, j) = ArmPosition(i, j) - ArmCentroid(j);
            EyeDeviation(i, j) = EyePosition(i, j) - EyeCentroid(j);
        }
    }

    Eigen::MatrixXf H(3, 3); 
    H = EyeDeviation.transpose() * ArmDeviation; 
            
    Eigen::JacobiSVD<Eigen::MatrixXf> SVD(H, Eigen::ComputeThinU | Eigen::ComputeThinV); 
  
    Rotation = SVD.matrixV() * SVD.matrixU().transpose(); 
    Translation = ArmCentroid - Rotation * EyeCentroid; 

    Homogeneous = Eigen::Matrix4f::Zero();
    Homogeneous.block<3, 3>(0, 0) = Rotation;
    Homogeneous.block<3, 1>(0, 3) = Translation;
    Homogeneous(3, 3) = 1.0;
    std::cout.setf(std::ios::showpoint);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout << std::setprecision(6) << Homogeneous << "\n";
}

HandEyeCalibration::HandEyeCalibration(char* file)
{
    if(!ParseCalibrationFile(file))
        return;
    else
        LeastSquareAlgorithm();
}

int main(int argc, char** argv)
{
    if(argc!=2)
    {
        printf("Please enter the address of calibration.txt. \n");
        return -1;
    }
    HandEyeCalibration HandEyeCalibration(argv[1]);
    return 0;
}
