#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid(int width, int height);
	float getCell(int x, int y);
	void setCell(int x, int y, float value);
	float heatEquation(int x, int y, float coefficient, float timestep);
	void zeroBoundary();
	void draw(DrawingUtilitiesClass *duc);
	~Grid();

private:
	vector<float> *data;
	int width;
	int height;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timestep);
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step
	int width;
	int height;
};

#endif