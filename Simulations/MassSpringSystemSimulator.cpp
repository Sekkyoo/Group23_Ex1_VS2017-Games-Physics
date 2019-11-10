#include "MassSpringSystemSimulator.h"

const double d =2;

MassSpringSystemSimulator::MassSpringSystemSimulator() {

	m_iIntegrator = EULER;
	
	ResetSystem();
	InitSetUps();
}

void MassSpringSystemSimulator::ResetSystem() {
	m_fMass = 1;
	m_fStiffness = 5;
	m_fDamping = 0.5;

	m_externalForce = *(new Vec3());

	m_massPointCount = 0;
	m_springCount = 0;

	m_massPoints = new MassPoint[fieldBuffer];
	m_massPiontLimit = fieldBuffer;
	m_springs = new Spring[fieldBuffer];
	m_springLimit = fieldBuffer;
}

void MassSpringSystemSimulator::InitSetUps() {
	m_systems = new MassSpringSystem[5];
	m_systems[0] = CreateSetUp1();
	m_systems[1] = CreateSetUp2();
	m_systems[2] = CreateSetUp3();
	m_systems[3] = CreateSetUp4();
	m_systems[4] = CreateExtraSetUp();
}

void MassSpringSystemSimulator::ChangeSetUp(MassSpringSystem system) {
	m_massPoints = system.massPoints;
	m_massPointCount = system.massPointCount;
	m_massPiontLimit = system.massPointCount;
	m_springs = system.springs;
	m_springCount = system.springCount;
	m_springLimit = system.springCount;
	m_fDamping = system.damping;
	m_fMass = system.mass;
	m_fStiffness = system.stiffness;
	m_externalForce = system.externalForce;

	m_iIntegrator = system.startIntegration;

	for (int i = 0; i < m_massPointCount; i++)
	{
		m_massPoints[i].position = m_massPoints[i].orignalPosition;
		m_massPoints[i].Velocity = m_massPoints[i].orignalVelocity;
	}
}

const char * MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1, Demo 2, Demo 3, Demo 4, Demo Extra";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Integration", TwDefineEnumFromString("Integration", "Euler,Leapfrog,Midpoint"), &m_iIntegrator, EULER);
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "Use Gravity", TW_TYPE_BOOLCPP, &m_useGravity, false);
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_gravity, "");
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	if (!m_systems[m_testCase].draw)
		return;

	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	for (int i = 0; i < m_massPointCount; i++)
	{
		DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 0), 100, 0.6*Vec3(1, 1, 0));
		DUC->drawSphere(m_massPoints[i].position, Vec3(0.01, 0.01, 0.01));
	}

	for (int i = 0; i < m_springCount; i++)
	{
		//DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->beginLine();
		DUC->drawLine(m_massPoints[m_springs[i].masspoint1].position, Vec3(0.2, 0.2, 1), m_massPoints[m_springs[i].masspoint2].position, Vec3(0.2, 0.2, 1));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_testCase = testCase;
	ChangeSetUp(m_systems[m_testCase]);
	cout << "Integration mode: " + std::to_string(m_iIntegrator) << endl;


	switch (testCase)
	{
	case 0:
		m_useGravity = false;
		simulateOneTimestepOnConsole(0.1);
		break;
	case 1: case 2:
		m_useGravity = false;
		simulateOneTimestepOnConsole(0.05);
		break;
	case 3:
		m_useGravity = true;
		break;
	case 4:
		m_useGravity = false;
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)

	//m_externalForce = Vec3(0,m_gravity,0);// m_systems[m_iIntegrator].externalForce;
	if (m_useGravity)
		m_externalForce += Vec3(0, m_gravity, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);

		m_mouseOffset = inputView * 0.00001;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	if (!m_systems[m_testCase].simulate)
		return;

	externalForcesCalculations(timeStep);

	switch (m_iIntegrator)
	{
	case EULER:
		stepEuler(timeStep);
		break;
	case LEAPFROG:
		stepLeapFrog(timeStep);
		break;
	case MIDPOINT:
		stepMidPoint(timeStep);
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::simulateOneTimestepOnConsole(float timeStep) {
	externalForcesCalculations(timeStep);
	cout << "Calculating the position and velocity after one a time step of " + std::to_string(timeStep) + " with integration type: ";
	switch (m_iIntegrator)
	{
	case EULER:
		cout << "Euler:" << endl;
		break;
	case LEAPFROG:
		cout << "Leapfrog:" << endl;
		break;
	case MIDPOINT:
		cout << "Midpoint:" << endl;
		break;
	default:
		break;
	}

	for (int i = 0; i < m_massPointCount; i++)
	{
		cout << "point " + std::to_string(i) + " position " + m_massPoints[i].position.toString() + " velocity " + m_massPoints[i].Velocity.toString() << endl;
	}

	switch (m_iIntegrator)
	{
	case EULER:
		stepEuler(timeStep);
		break;
	case LEAPFROG:
		stepLeapFrog(timeStep);
		break;
	case MIDPOINT:
		stepMidPoint(timeStep);
		break;
	default:
		break;
	}

	cout << "After the step" << endl;
	for (int i = 0; i < m_massPointCount; i++)
	{
		cout << "point " + std::to_string(i) + " position " + m_massPoints[i].position.toString() + " velocity " + m_massPoints[i].Velocity.toString() << endl;
	}
	cout << endl << endl;
}

void MassSpringSystemSimulator::stepEuler(float timeStep) {
	Vec3* accelorations = computeAcceloration();
	for (int i = 0; i < m_massPointCount; i++)
	{
		if (m_massPoints[i].isFixed)
			continue;

		m_massPoints[i].position += m_massPoints[i].Velocity * timeStep;
		if(m_systems[m_testCase].clamp)
			m_massPoints[i].position = ClampVector(m_massPoints[i].position, 0.5f, -0.5f, i);
	

		m_massPoints[i].Velocity += accelorations[i] * timeStep;
	}
}

void MassSpringSystemSimulator::stepLeapFrog(float timeStep) {
	Vec3* accelorations = computeAcceloration();
	for (int i = 0; i < m_massPointCount; i++)
	{

		if (m_massPoints[i].isFixed)
			continue;

		m_massPoints[i].Velocity += accelorations[i] * timeStep;


		m_massPoints[i].position += m_massPoints[i].Velocity * timeStep + m_mouseOffset;
		if (m_systems[m_testCase].clamp)
			m_massPoints[i].position = ClampVector(m_massPoints[i].position, 0.5f, -0.5f, i);


	}
}

void MassSpringSystemSimulator::stepMidPoint(float timeStep) {

	// Calculate the position at the midstep.
	Vec3* posOfMidstep = new Vec3[m_massPointCount];
	for (int i = 0; i < m_massPointCount; i++)
	{
		if (m_massPoints[i].isFixed)
			continue;
		posOfMidstep[i] = m_massPoints[i].position + 0.5 * timeStep * m_massPoints[i].Velocity;
	}

	// Calculatte the velocity at the midstep.
	Vec3* acc = computeAcceloration();
	Vec3* velOfMidstep = new Vec3[m_massPointCount];
	for (int i = 0; i < m_massPointCount; i++)
	{
		if (m_massPoints[i].isFixed)
			continue;
		velOfMidstep[i] = m_massPoints[i].Velocity + 0.5 * timeStep * acc[i];
	}

	// Calculate new pos.
	for (int i = 0; i < m_massPointCount; i++)
	{
		if (m_massPoints[i].isFixed)
			continue;
		m_massPoints[i].position = (m_massPoints[i].position + timeStep * velOfMidstep[i]) + m_mouseOffset;
		if(m_systems[m_testCase].clamp)
			m_massPoints[i].position = ClampVector(m_massPoints[i].position, 0.5f, -0.5f, i);
	}

	// Calculate new velocity.
	// Update acceleration with midstep positions.
	acc = computeAcceloration(posOfMidstep);
	for (int i = 0; i < m_massPointCount; i++)
	{
		if (m_massPoints[i].isFixed)
			continue;
		m_massPoints[i].Velocity = (m_massPoints[i].Velocity + timeStep * acc[i]) * (1 - m_fDamping);
	}
}

Vec3 MassSpringSystemSimulator::ClampVector(Vec3 input, float minimum, float maximum, int pointIndex)
{
	cout << m_systems[m_testCase].clamp << endl;
	if (!m_systems[m_testCase].clamp)
		return input;
	input.makeFloor(Vec3(minimum, minimum, minimum));
	input.makeCeil(Vec3(maximum, maximum, maximum));
	if (input.y == minimum)
		m_massPoints[pointIndex].Velocity.y = 0;
	return input;
}

Vec3* MassSpringSystemSimulator::computeAcceloration()
{
	Vec3* points = new Vec3[m_massPointCount];
	for (int i = 0; i < m_massPointCount; i++)
	{
		points[i] = m_massPoints[i].position;
	}
	return computeAcceloration(points);
}

Vec3* MassSpringSystemSimulator::computeAcceloration(Vec3* points)
{
	Vec3* acc = new Vec3[m_massPointCount];
	for (int i = 0; i < m_springCount; i++)
	{
		// cout << "Spring: " + std::to_string(i) << endl;
		Vec3 dir = points[m_springs[i].masspoint1] - points[m_springs[i].masspoint2];
		float distance = sqrt(pow(dir.x, 2) + pow(dir.y, 2) + pow(dir.z, 2));

		float force = -m_fStiffness * (distance - m_springs->initialLength)/ m_fMass;
		Vec3 dirNorm = Vec3(dir.x/ distance, dir.y/ distance, dir.z/ distance);
		acc[m_springs[i].masspoint1] += force * dirNorm;
		acc[m_springs[i].masspoint2] -= force * dirNorm;
	}

	for (int i = 0; i < m_massPointCount; i++)
	{
		acc[i] += m_externalForce;
	}
	return acc;
}

void MassSpringSystemSimulator:: onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	if (m_massPointCount == m_massPiontLimit)
	{
		MassPoint * newMassPoints = new MassPoint[m_massPiontLimit + fieldBuffer];
		for (int i = 0; i < m_massPiontLimit; i++)
		{
			newMassPoints[i] = m_massPoints[i];
		}
		m_massPoints = newMassPoints;
		m_massPiontLimit += fieldBuffer;
	}

	m_massPoints[m_massPointCount] = {position, position, Velocity, Velocity, isFixed};
	m_massPointCount++;
	return m_massPointCount - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (m_springCount == m_springCount)
	{
		Spring * newsprings = new Spring[m_massPiontLimit + fieldBuffer];
		for (int i = 0; i < m_massPiontLimit; i++)
		{
			newsprings[i] = m_springs[i];
		}
		m_springs = newsprings;
		m_massPiontLimit += fieldBuffer;
	}

	m_springs[m_springCount] = { masspoint1, masspoint2, initialLength};
	m_springCount++;
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_massPointCount;
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_springCount;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return m_massPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return m_massPoints[index].Velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	m_externalForce = force;
}

MassSpringSystem MassSpringSystemSimulator::CreateSetUp1() {
	ResetSystem();

	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);

	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);
	setIntegrator(EULER);

	return { m_massPoints, 2, m_springs, 1, m_fDamping, m_fStiffness, m_fMass, Vec3(0, 0, 0), false, false, m_iIntegrator, false};
}

MassSpringSystem MassSpringSystemSimulator::CreateSetUp2() {
	MassSpringSystem mss = CreateSetUp1();
	mss.draw = true;
	mss.startIntegration = EULER;
	return mss;
}

MassSpringSystem MassSpringSystemSimulator::CreateSetUp3() {
	ResetSystem();

	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);

	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);

	setIntegrator(MIDPOINT);

	return { m_massPoints, 2, m_springs, 1, m_fDamping, m_fStiffness, m_fMass, m_externalForce, true, false, m_iIntegrator, false};
}

MassSpringSystem MassSpringSystemSimulator::CreateSetUp4() {
	ResetSystem();

	// Ikosaeder.
	float size = 0.1f;
	float mue = 0.61803f * size;

	int p0 = addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(0, 0, 0), false);
	int p1 = addMassPoint(Vec3(0, size, mue), Vec3(0, 0, 0), false);
	int p2 = addMassPoint(Vec3(0, size, -mue), Vec3(0, 0, 0), false);
	int p3 = addMassPoint(Vec3(0, -size, mue), Vec3(0, 0, 0), false);
	int p4 = addMassPoint(Vec3(0, -size, -mue), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(mue, 0, size), Vec3(0, 0, 0), false);
	int p6 = addMassPoint(Vec3(-mue, 0, size), Vec3(0, 0, 0), false);
	int p7 = addMassPoint(Vec3(mue, 0, -size), Vec3(0, 0, 0), false);
	int p8 = addMassPoint(Vec3(-mue, 0, -size), Vec3(0, 0, 0), false);
	int p9 = addMassPoint(Vec3(size, mue, 0), Vec3(0, 0, 0), false);
	int p10 = addMassPoint(Vec3(size, -mue, 0), Vec3(0, 0, 0), false);
	int p11 = addMassPoint(Vec3(-size, mue, 0), Vec3(0, 0, 0), false);
	int p12 = addMassPoint(Vec3(-size, -mue, 0), Vec3(0, 0, 0), false);

	//Springs.
	addSpring(p1, p2, 2 * mue);
	addSpring(p3, p4, 2 * mue);
	addSpring(p5, p6, 2 * mue);
	addSpring(p7, p8, 2 * mue);
	addSpring(p9, p10, 2 * mue);
	addSpring(p11, p12, 2 * mue);

	addSpring(p1, p5, 2 * mue);
	addSpring(p1, p6, 2 * mue);
	addSpring(p2, p7, 2 * mue);
	addSpring(p2, p8, 2 * mue);
	addSpring(p3, p5, 2 * mue);
	addSpring(p3, p6, 2 * mue);
	addSpring(p4, p7, 2 * mue);
	addSpring(p4, p8, 2 * mue);

	addSpring(p5, p9, 2 * mue);
	addSpring(p5, p10, 2 * mue);
	addSpring(p6, p11, 2 * mue);
	addSpring(p6, p12, 2 * mue);
	addSpring(p7, p9, 2 * mue);
	addSpring(p7, p10, 2 * mue);
	addSpring(p8, p11, 2 * mue);
	addSpring(p8, p12, 2 * mue);

	addSpring(p9, p1, 2 * mue);
	addSpring(p9, p2, 2 * mue);
	addSpring(p10, p3, 2 * mue);
	addSpring(p10, p4, 2 * mue);
	addSpring(p11, p1, 2 * mue);
	addSpring(p11, p2, 2 * mue);
	addSpring(p12, p3, 2 * mue);
	addSpring(p12, p4, 2 * mue);

	// From centre.
	addSpring(p0, p1, size * 1.7557f);
	addSpring(p0, p2, size * 1.7557f);
	addSpring(p0, p3, size * 1.7557f);
	addSpring(p0, p4, size * 1.7557f);
	addSpring(p0, p5, size * 1.7557f);
	addSpring(p0, p6, size * 1.7557f);
	addSpring(p0, p7, size * 1.7557f);
	addSpring(p0, p8, size * 1.7557f);
	addSpring(p0, p9, size * 1.7557f);
	addSpring(p0, p10, size * 1.7557f);
	addSpring(p0, p11, size * 1.7557f);
	addSpring(p0, p12, size * 1.7557f);

	setIntegrator(MIDPOINT);
	setDampingFactor(0.005);
	setStiffness(8000);
	setMass(0.7);

	return {m_massPoints, 13, m_springs, 42, m_fDamping, m_fStiffness, m_fMass, m_externalForce, true, true, m_iIntegrator, true};
}

MassSpringSystem MassSpringSystemSimulator::CreateExtraSetUp() {
	ResetSystem();

	// Ikosaeder.
	float size = 0.1f;
	float mue = 0.61803f * size;

	int p0 = addMassPoint(Vec3(0.5f, 0.5f, 0.5f), Vec3(0, 0, 0), true);
	int p6 = addMassPoint(Vec3(0.5f, 0.5f, -0.5f), Vec3(0, 0, 0), true);
	int p7 = addMassPoint(Vec3(-0.5f, 0.5f, 0.5f), Vec3(0, 0, 0), true);
	int p8 = addMassPoint(Vec3(-0.5f, 0.5f, -0.5f), Vec3(0, 0, 0), true);

	int p1 = addMassPoint(Vec3(0.0f, -0.2f, 0.0f), Vec3(0, 0, 0), false);

	int p2 = addMassPoint(Vec3(0.1f, 0.0f, 0.1f), Vec3(0, 0, 0), false);
	int p3 = addMassPoint(Vec3(0.1f, 0.0f, -0.1f), Vec3(0, 0, 0), false);
	int p4 = addMassPoint(Vec3(-0.1f, 0.0f, 0.1f), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(-0.1f, 0.0f, -0.1f), Vec3(0, 0, 0), false);
	
	addSpring(p0, p2, 0.2f);
	addSpring(p6, p3, 0.2f);
	addSpring(p7, p4, 0.2f);
	addSpring(p8, p5, 0.2f);
	
	addSpring(p1, p2, 0.244949f);
	addSpring(p1, p3, 0.244949f);
	addSpring(p1, p4, 0.244949f);
	addSpring(p1, p5, 0.244949f);

	addSpring(p2, p3, 0.2f);
	addSpring(p3, p5, 0.2f);
	addSpring(p5, p4, 0.2f);
	addSpring(p4, p2, 0.2f);


	setIntegrator(MIDPOINT);
	setDampingFactor(0.005);
	setStiffness(8000);
	setMass(1);

	return { m_massPoints, 9, m_springs, 12, m_fDamping, m_fStiffness, m_fMass, m_externalForce, true, true, m_iIntegrator, true };
}	