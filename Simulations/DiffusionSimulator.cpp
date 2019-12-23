#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(int width, int height) {
	this->width = width;
	this->height = height;
	this->data = new std::vector<float>(width * height);
	fill(this->data->begin(), this->data->end(), 0.0f);
}

float Grid::getCell(int x, int y) {
	if (x < 0 || x >= width || y < 0 || y >= height) return 0.0f;
	return data->at(y * width + x);
}

void Grid::setCell(int x, int y, float value) {
	data->at(y * width + x) = value;
}

void Grid::zeroBoundary() {
	for (int x = 0; x < width; x++) {
		setCell(x, 0, 0.0f);
		setCell(x, height - 1, 0.0f);
	}
	for (int y = 0; y < height; y++) {
		setCell(0, y, 0.0f);
		setCell(width - 1, y, 0.0f);
	}
}

float Grid::heatEquation(int x, int y, float coefficient, float timestep) {
	float cellVal = getCell(x, y);
	float dxx = (getCell(x + 1, y) - (2.0f * cellVal) + getCell(x - 1, y)) / 2.0f;
	float dyy = (getCell(x, y + 1) - (2.0f * cellVal) + getCell(x, y - 1)) / 2.0f;
	return cellVal + coefficient * (dxx + dyy);
}

void Grid::draw(DrawingUtilitiesClass *duc) {
	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			float value = getCell(x, y);
			float colOffset = value * 0.1f;
			duc->setUpLighting(Vec3(1.0, max(0.0, 1.0 - colOffset), max(0.0, 1.0 - colOffset)), Vec3(), 0.5, Vec3());
			duc->drawSphere(Vec3(x, y, 0.0) * 0.1, .05);
		}
	}
}

Grid::~Grid() {
	delete this->data;
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	
	width = 5;
	height = 5;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "width", TW_TYPE_INT32, &width, "min=3");
	TwAddVarRW(DUC->g_pTweakBar, "height", TW_TYPE_INT32, &height, "min=3");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	T = new Grid(width, height);
	T->setCell(width / 2, height / 2, 100.0);
	T->setCell(width / 4, height / 4, 100.0);
	T->zeroBoundary();

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timestep) {
	Grid* newT = new Grid(width, height);

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			newT->setCell(x, y, T->heatEquation(x, y, .01f, timestep));
		}
	}
	
	newT->zeroBoundary();
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	T->draw(DUC);
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
