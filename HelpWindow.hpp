#ifndef HELPWINDOW_HPP
#define HELPWINDOW_HPP

#include "Configs.hpp"

//-- Help Window Class
class HelpWindow {
    private:
        Mat helpWin;
        bool behaviourMode;
        //-- Config Display Values
        double Scale;
        double middleLength;
        double middleWidth;
        //-- Keys
        double keySize;
        double keyLineSize;
        double keyPaddingLength;
        double keyPaddingWidth;
        double keyPositionX;
        double keyPositionY;
        double keyPressedValue;
        double keyFreeValue;
        double textPositionLength;
        double textPositionWidth;
        double keyTextBlue;
        double keyTextGreen;
        double keyTextRed;
        double keyInfoPositionX;
        double keyInfoPositionY;
        double spaceBetweenLines;
        //-- Activity Circle
        double activityCirclePosition;
        double activityStatusPosition;
        //-- Agent
        double agentCenterX;
        double agentCenterY;
        //-- Velocity and Position
        double speedPositionLength;
        double xPosLength;
        double yPosLength;
        double thetaPosLength;
        double xPosWidth;
        double yPosWidth;
        double thetaPosWidth;
        double vXPosLength;
        double vYPosLength;
        double vThetaPosLength;
        double vXPosWidth;
        double vYPosWidth;
        double vThetaPosWidth;
        string xInfo;
        string yInfo;
        string thetaInfo;
        string vXInfo;
        string vYInfo;
        string vThetaInfo;
        double modelX;
        double modelY;
        double modelTheta;
        double agentVX;
        double agentVY;
        double agentVTheta;
        int maxColor;
        double vMoveVectorX;
        double vMoveVectorY;
        double vRotateVectorX;
        double vRotateVectorY;
        double xDestPosLength;
        double yDestPosLength;
        double thetaDestPosLength;
        double xDestPosWidth;
        double yDestPosWidth;
        double thetaDestPosWidth;
        //-- Status Circle
        double statusPosition;
        double speedPositionWidth;
        double activityCircleRadius;
        string status;
        string error;
        string speedStatus;
        int activeValue;
        int stoppedValue;
        bool active;
        int state;
        int fadingFluency;
        int fadingSpeed;
        int counter;
        int statusCircleSize;
        double destinationX;
        double destinationY;
        double destX;
        double destY;
        double destTheta;
        string destXInfo;
        string destYInfo;
        string destThetaInfo;
        bool changePos;
        int posCounter;
        double agentSize;
    public:
        void updateHelpWindow(bool);
        HelpWindow();
        //-- Update Status of Changes
        void updateStatus(int, double);
        void updateError(string);
        int Fader();
        void resetError();
        void viewPosition(double, double, double);
        void viewVelocity(double, double, double);
        void setMode();
        void setDestination(double, double, double);
};

#endif // HELPWINDOW_HPP
