#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void drawPointCloud();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofxKinect kinect;
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    
    vector<ofMesh> totalMesh;
    int lastSecond = 0; //used to track time
    int w; //window width
    int h; //window height
    
    //recording settings
    int minFigureDistance; //min distance from camera things being recorded should be
    int maxFigureDistance; //min distance from camera things being recorded should be
    
    //sampling settings
    int lastSampleLoc[2]; //so we know where we should sample around
    int nearby;//how far to sample the neighbor
    int minConnectionDistance; //how far can we connect vertexes
    int maxConnectionDistance; //how far can we connect vertexes
    
};
