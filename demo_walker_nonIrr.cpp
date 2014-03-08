#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"

using namespace chrono;

double timestep(0.0004);
double simDuration(10);
int everyFrame(40);

double speed(2); // 1:persistent contact
double chassisL(15.0);
double chassisW(2.0);
double legW(1.0);
double legL(2.0);
double footH(0.5);
double footW(1.5);
double footL(2.0);
double axleL(3.0);

double boxW(12);
double boxL(50);
double boxH(2);

double boxDrop(2+boxH); // used to lower elevation of box and terrain

double particleDiameter(0.2); // diameter of terrain particle


void createTerrain(ChSystem & sys, double dia, double W, double H, double L)
{
	int x = W/dia-3;
	int y = H/dia;
	int z = L/dia-3;
	double mass = 0.05 * 4/3*CH_C_PI*pow(particleDiameter/2.0,3.0);
	double inertia = 0.4*mass*particleDiameter*particleDiameter/4.0;

	ChSharedBodyPtr particle;
	for (int i=3; i<x; i++)
	{
		for (int j=0; j<y; j++)
		{
			for (int k=3; k<z; k++)
			{
				particle=ChSharedBodyPtr(new ChBody);
				particle->SetPos(ChVector<>(-W/2.0,-boxDrop,-(chassisL/2.0)-2.0) + ChVector<>(i+CHrandom()-0.5,j,k+CHrandom()-0.5) * dia);
				particle->SetMass(mass); // density * volume
				particle->SetInertiaXX(ChVector<>(inertia,inertia,inertia));
				particle->GetCollisionModel()->AddSphere(dia/2.0);
				particle->SetCollide(true);
				particle->SetPos_dt(ChVector<>(CHrandom()-0.5,-CHrandom(),CHrandom()-0.5)*10 );
				sys.AddBody(particle);
			}
		}
	}
}


int main(int argc, char* argv[])
{

	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	{
		// The physical system: it contains all physical objects.
		ChSystem my_system;

		ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);

		// Create rigid bodies
		GetLog() << "Creating Bodies...";

		// create pointers to bodies
		ChSharedBodyPtr  floor(new ChBody);  
		ChSharedBodyPtr  wallL(new ChBody);  
		ChSharedBodyPtr  wallR(new ChBody);  
		ChSharedBodyPtr  wallF(new ChBody);  
		ChSharedBodyPtr  wallB(new ChBody);  

		ChSharedBodyPtr  chassis(new ChBody); 

		ChSharedBodyPtr  axle_F(new ChBody); 
		ChSharedBodyPtr  axle_C(new ChBody); 
		ChSharedBodyPtr  axle_R(new ChBody);

		ChSharedBodyPtr  leg_FR(new ChBody); 
		ChSharedBodyPtr  leg_FL(new ChBody); 
		ChSharedBodyPtr  leg_CR(new ChBody); 
		ChSharedBodyPtr  leg_CL(new ChBody); 
		ChSharedBodyPtr  leg_RR(new ChBody); 
		ChSharedBodyPtr  leg_RL(new ChBody); 

		ChSharedBodyPtr  foot_FR(new ChBody); 
		ChSharedBodyPtr  foot_FL(new ChBody); 
		ChSharedBodyPtr  foot_CR(new ChBody); 
		ChSharedBodyPtr  foot_CL(new ChBody); 
		ChSharedBodyPtr  foot_RR(new ChBody); 
		ChSharedBodyPtr  foot_RL(new ChBody); 


		// Set initial position of the bodies (center of mass)
		floor->SetPos(ChVector<>(0,				  -3-boxDrop,	boxL/2.0-(chassisL/2.0)-2.0) );
		wallL->SetPos(ChVector<>(-boxW/2.0-0.5, boxH/2.0-boxDrop,	boxL/2.0-(chassisL/2.0)-2.0) );
		wallR->SetPos(ChVector<>( boxW/2.0+0.5, boxH/2.0-boxDrop,	boxL/2.0-(chassisL/2.0)-2.0) );
		wallF->SetPos(ChVector<>(0,			boxH/2.0-boxDrop,	boxL-chassisL/2.0-2.0+0.5) );
		wallB->SetPos(ChVector<>(0,			boxH/2.0-boxDrop,       -chassisL/2.0-2.0-0.5) );

		floor->SetBodyFixed(true);
		wallL->SetBodyFixed(true);
		wallR->SetBodyFixed(true);
		wallF->SetBodyFixed(true);
		wallB->SetBodyFixed(true);
		
		chassis->SetPos(VNULL);

		axle_F->SetPos(ChVector<>(0,0,chassisL/2));
		axle_C->SetPos(ChVector<>(0,0,0));
		axle_R->SetPos(ChVector<>(0,0,-chassisL/2));

		leg_FR->SetPos(ChVector<>( (axleL+legW)/2.0, -legL/2.0,  chassisL/2));
		leg_FL->SetPos(ChVector<>(-(axleL+legW)/2.0,  legL/2.0,  chassisL/2));
		leg_CR->SetPos(ChVector<>(-(axleL+legW)/2.0, -legL/2.0,  0));
		leg_CL->SetPos(ChVector<>( (axleL+legW)/2.0,  legL/2.0,  0));
		leg_RR->SetPos(ChVector<>( (axleL+legW)/2.0, -legL/2.0, -chassisL/2));
		leg_RL->SetPos(ChVector<>(-(axleL+legW)/2.0,  legL/2.0, -chassisL/2));

		foot_FR->SetPos(ChVector<>( (axleL+footW)/2.0, -(legL-footH/2.0),  chassisL/2+1));
		foot_FL->SetPos(ChVector<>(-(axleL+footW)/2.0,  (legL-footH/2.0),  chassisL/2-1));
		foot_CR->SetPos(ChVector<>(-(axleL+footW)/2.0, -(legL-footH/2.0),  1));
		foot_CL->SetPos(ChVector<>( (axleL+footW)/2.0,  (legL-footH/2.0), -1));
		foot_RR->SetPos(ChVector<>( (axleL+footW)/2.0, -(legL-footH/2.0), -chassisL/2+1));
		foot_RL->SetPos(ChVector<>(-(axleL+footW)/2.0,  (legL-footH/2.0), -chassisL/2-1));


		// Set masses
		chassis->SetMass(4.0);

		axle_F->SetMass(0.1);
		axle_C->SetMass(0.1);
		axle_R->SetMass(0.1);

		leg_FR->SetMass(0.1);
		leg_FL->SetMass(0.1);
		leg_CR->SetMass(0.1);
		leg_CL->SetMass(0.1);
		leg_RR->SetMass(0.1);
		leg_RL->SetMass(0.1);

		foot_FR->SetMass(0.1);
		foot_FL->SetMass(0.1);
		foot_CR->SetMass(0.1);
		foot_CL->SetMass(0.1);
		foot_RR->SetMass(0.1);
		foot_RL->SetMass(0.1);
		
		
		//
		// Create collision models
		//

		// box
		floor->GetCollisionModel()->AddBox(50,3/2.0,200, &VNULL);
		floor->SetCollide(true);
		wallL->GetCollisionModel()->AddBox(1, boxH, boxL);
		wallR->GetCollisionModel()->AddBox(1, boxH, boxL);
		wallF->GetCollisionModel()->AddBox(boxW, boxH, 1);
		wallB->GetCollisionModel()->AddBox(boxW, boxH, 1);
		wallL->SetCollide(true);
		wallR->SetCollide(true);
		wallF->SetCollide(true);
		wallB->SetCollide(true);
		/*
		floor->GetCollisionModel()->SetFamily(0);
		wallL->GetCollisionModel()->SetFamily(0);
		wallR->GetCollisionModel()->SetFamily(0);
		wallF->GetCollisionModel()->SetFamily(0);
		wallB->GetCollisionModel()->SetFamily(0);
		floor->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(0);
		wallL->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(0);
		wallR->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(0);
		wallF->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(0);
		wallB->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(0);
		*/

		chassis->GetCollisionModel()->AddBox(chassisW/2.0,1.0/2.0,chassisL/2.0, &VNULL);
		chassis->SetCollide(true);
		chassis->GetCollisionModel()->SetFamily(1);
		// Do not collide axles with chassis
		chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);

		axle_F->GetCollisionModel()->AddCylinder(0.5, axleL, 0.5, &VNULL, &ChMatrix33<double>(Q_from_AngZ(CH_C_PI/2.0)) );
		axle_C->GetCollisionModel()->AddCylinder(0.5, axleL, 0.5, &VNULL, &ChMatrix33<double>(Q_from_AngZ(CH_C_PI/2.0)) );
		axle_R->GetCollisionModel()->AddCylinder(0.5, axleL, 0.5, &VNULL, &ChMatrix33<double>(Q_from_AngZ(CH_C_PI/2.0)) );
		axle_F->SetCollide(true);
		axle_C->SetCollide(true);
		axle_R->SetCollide(true);
		// Chassis will not collide with the family of axles
		axle_F->GetCollisionModel()->SetFamily(2);
		axle_C->GetCollisionModel()->SetFamily(2);
		axle_R->GetCollisionModel()->SetFamily(2);

		leg_FR->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_FL->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_CR->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_CL->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_RR->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_RL->GetCollisionModel()->AddBox(legW/2.0, legL/2.0, 0.5/2.0, &VNULL);
		leg_FR->SetCollide(true);
		leg_FL->SetCollide(true);
		leg_CR->SetCollide(true);
		leg_CL->SetCollide(true);
		leg_RR->SetCollide(true);
		leg_RL->SetCollide(true);

		foot_FR->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_FL->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_CR->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_CL->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_RR->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_RL->GetCollisionModel()->AddBox(footW/2.0, footH/2.0, footL/2.0, &VNULL);
		foot_FR->SetCollide(true);
		foot_FL->SetCollide(true);
		foot_CR->SetCollide(true);
		foot_CL->SetCollide(true);
		foot_RR->SetCollide(true);
		foot_RL->SetCollide(true);



		// Add bodies to system
		my_system.AddBody(floor);
		my_system.AddBody(wallR);
		my_system.AddBody(wallL);
		my_system.AddBody(wallF);
		my_system.AddBody(wallB);

		my_system.AddBody(chassis);

		my_system.AddBody(axle_F);
		my_system.AddBody(axle_C);
		my_system.AddBody(axle_R);

		my_system.AddBody(leg_FR);
		my_system.AddBody(leg_FL);
		my_system.AddBody(leg_CR);
		my_system.AddBody(leg_CL);
		my_system.AddBody(leg_RR);
		my_system.AddBody(leg_RL);

		my_system.AddBody(foot_FR);
		my_system.AddBody(foot_FL);
		my_system.AddBody(foot_CR);
		my_system.AddBody(foot_CL);
		my_system.AddBody(foot_RR);
		my_system.AddBody(foot_RL);

		// Create lattice of spheres as gruanular terrain
		createTerrain(my_system, particleDiameter, boxW, boxH, boxL);

		GetLog() << "DONE\n";



		// Create links between bodies
		GetLog() << "Creating Links...";

		// attach feet to legs
		ChSharedPtr<ChLinkLockLock> link_FR(new ChLinkLockLock); 
		link_FR->Initialize(foot_FR, leg_FR, ChCoordsys<>(VNULL));
		my_system.AddLink(link_FR);

		ChSharedPtr<ChLinkLockLock> link_FL(new ChLinkLockLock); 
		link_FL->Initialize(foot_FL, leg_FL, ChCoordsys<>(VNULL));
		my_system.AddLink(link_FL);

		ChSharedPtr<ChLinkLockLock> link_CR(new ChLinkLockLock); 
		link_CR->Initialize(foot_CR, leg_CR, ChCoordsys<>(VNULL));
		my_system.AddLink(link_CR);

		ChSharedPtr<ChLinkLockLock> link_CL(new ChLinkLockLock); 
		link_CL->Initialize(foot_CL, leg_CL, ChCoordsys<>(VNULL));
		my_system.AddLink(link_CL);

		ChSharedPtr<ChLinkLockLock> link_RR(new ChLinkLockLock); 
		link_RR->Initialize(foot_RR, leg_RR, ChCoordsys<>(VNULL));
		my_system.AddLink(link_RR);

		ChSharedPtr<ChLinkLockLock> link_RL(new ChLinkLockLock); 
		link_RL->Initialize(foot_RL, leg_RL, ChCoordsys<>(VNULL));
		my_system.AddLink(link_RL);

		// attach legs to axles
		ChSharedPtr<ChLinkLockLock> axle_FR(new ChLinkLockLock); 
		axle_FR->Initialize(leg_FR, axle_F, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_FR);

		ChSharedPtr<ChLinkLockLock> axle_FL(new ChLinkLockLock); 
		axle_FL->Initialize(leg_FL, axle_F, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_FL);

		ChSharedPtr<ChLinkLockLock> axle_CR(new ChLinkLockLock); 
		axle_CR->Initialize(leg_CR, axle_C, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_CR);

		ChSharedPtr<ChLinkLockLock> axle_CL(new ChLinkLockLock); 
		axle_CL->Initialize(leg_CL, axle_C, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_CL);

		ChSharedPtr<ChLinkLockLock> axle_RR(new ChLinkLockLock); 
		axle_RR->Initialize(leg_RR, axle_R, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_RR);

		ChSharedPtr<ChLinkLockLock> axle_RL(new ChLinkLockLock); 
		axle_RL->Initialize(leg_RL, axle_R, ChCoordsys<>(VNULL));
		my_system.AddLink(axle_RL);

		GetLog() << "DONE\n";



		// Create engine between axles and chassis
		GetLog() << "Creating Motors...";

		ChSharedPtr<ChLinkEngine> eng_F(new ChLinkEngine);
		eng_F->Initialize(axle_F, chassis, 
			ChCoordsys<>(ChVector<>(0, 0, chassisL/2) , Q_from_AngY(CH_C_PI/2) ) );
		eng_F->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		eng_F->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(eng_F->Get_spe_funct()))
			mfun->Set_yconst(speed); // rad/s  angular speed
		my_system.AddLink(eng_F);

		ChSharedPtr<ChLinkEngine> eng_C(new ChLinkEngine);
		eng_C->Initialize(axle_C, chassis, 
			ChCoordsys<>(ChVector<>(0, 0, 0) , Q_from_AngY(CH_C_PI/2) ) );
		eng_C->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		eng_C->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(eng_C->Get_spe_funct()))
			mfun->Set_yconst(speed); // rad/s  angular speed
		my_system.AddLink(eng_C);

		ChSharedPtr<ChLinkEngine> eng_R(new ChLinkEngine);
		eng_R->Initialize(axle_R, chassis, 
			ChCoordsys<>(ChVector<>(0, 0, -chassisL/2) , Q_from_AngY(CH_C_PI/2) ) );
		eng_R->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		eng_R->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
		if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(eng_R->Get_spe_funct()))
			mfun->Set_yconst(speed); // rad/s  angular speed
		my_system.AddLink(eng_R);

		GetLog() << "DONE\n";

		/*
		// View system hierarchy and constraints

		GetLog() << "\n\nSystem Hierarchy:\n";
		my_system.ShowHierarchy(GetLog());
		
		GetLog() << "\n\nConstraints:\n";
		ChSystem::IteratorLinks myiter = my_system.IterBeginLinks();
		while (myiter != my_system.IterEndLinks())
		{ 
		GetLog() << "   Link class: " << (*myiter)->GetRTTI()->GetName() << "  , leaves n.DOFs: "  << (*myiter)->GetLeftDOF() << "\n";
		++myiter;
		}
		*/

		// Prepare the physical system for the simulation
		my_system.SetIntegrationType(ChSystem::INT_ANITESCU); 
		my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
		my_system.SetMaxiter(50);
		my_system.SetIterLCPmaxItersSpeed(50);


		// OK! NOW GET READY FOR THE DYNAMICAL SIMULATION!
		GetLog() << "Running Simulation...\n";

		// A very simple simulation loop..
		double simTime(0);
		int currFrame(0);

		while (simTime < simDuration)
		{
			simTime += timestep; 

			// PERFORM SIMULATION UP TO chronoTime
			my_system.DoFrameDynamics(simTime);

			if (currFrame%everyFrame == 0 )
			{

				// Output data to files
				char padnumber[256];
				sprintf(padnumber, "%d", (currFrame/everyFrame + 10000));
				char filename[256];
				// filepath (/walkerdata) must exist before executing
				sprintf(filename, "%s/pos%s.txt", "walkerdata", padnumber + 1);

				// create output file
				ChStreamOutAsciiFile output(filename);
				
				// i=5 => skip floor and walls
				for (int i = 5; i < my_system.Get_bodylist()->size(); i++) {
				ChBody* abody = (ChBody*) my_system.Get_bodylist()->at(i);
				ChVector<> bodPos = abody->GetPos();
						ChQuaternion<> bodRot = abody->GetRot();
						output << (float)bodPos.x << ",";
						output << (float)bodPos.y << ",";
						output << (float)bodPos.z << ",";
						output << (float)bodRot.e0 << ",";
						output << (float)bodRot.e1 << ",";
						output << (float)bodRot.e2 << ",";
						output << (float)bodRot.e3 << ",\n";
				}	

			}
			currFrame++;
			GetLog() << "Completed Step " << currFrame << "\n";
		}
		
		GetLog() << "DONE\n";
	}


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


