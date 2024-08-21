unit HexapodSim;

{$MODE Delphi}

interface

uses Kraft,UnitDemoScene, Math, Crt;

const Count={4}12;
      Count2={4}2;
      Count3={4}8;

type TDemoSceneCatapult=class(TDemoScene)
      public
       RigidBodyFloor:TKraftRigidBody;
       ShapeFloorPlane:TKraftShapePlane;
        RigidBodyBox:array[0..Count2-1] of TKraftRigidBody;
        RigidBodyCapsule:array[0..Count-1] of TKraftRigidBody;
        RigidBodyMassBox:array[0..Count3-1] of TKraftRigidBody;
        Joint{1} : array[0..Count-1] of TKraftConstraintJointHinge;
       constructor Create(const AKraftPhysics:TKraft); override;
       destructor Destroy; override;
       procedure Step(const DeltaTime:double); override;
     end;

var CNormal : array[1..3] of SINGLE;

    Iteration : Int64;

    FX1, FY1, FZ1, FX2, FY2, FZ2 : SINGLE;

    MuscleLen, MuscleAmp, SpringStretch1, SpringStretch2, SpringStretch3, SpringStretch4 : SINGLE;

    FMotor : SINGLE;

implementation

uses UnitFormMain;

PROCEDURE FindNormal(v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z : SINGLE);
const	x = 1;
      y = 2;
      z = 3;
var	temp_v1, temp_v2 : array[1..3] of SINGLE;
      temp_lenght : real;
begin

temp_v1[x] := v1x - v2x;
temp_v1[y] := v1y - v2y;
temp_v1[z] := v1z - v2z;

temp_v2[x] := v2x - v3x;
temp_v2[y] := v2y - v3y;
temp_v2[z] := v2z - v3z;

// calculate cross product
CNormal[x] := temp_v1[y]*temp_v2[z] - temp_v1[z]*temp_v2[y];
CNormal[y] := temp_v1[z]*temp_v2[x] - temp_v1[x]*temp_v2[z];
CNormal[z] := temp_v1[x]*temp_v2[y] - temp_v1[y]*temp_v2[x];

// normalize normal
temp_lenght :=	(CNormal[x]*CNormal[x])+
      	(CNormal[y]*CNormal[y])+
      	(CNormal[z]*CNormal[z]);

temp_lenght := sqrt(temp_lenght);

// prevent n/0
if temp_lenght = 0 then temp_lenght := 1;

CNormal[x] := CNormal[x] / temp_lenght;
CNormal[y] := CNormal[y] / temp_lenght;
CNormal[z] := CNormal[z] / temp_lenght;

end;

procedure CalcSpringForces(X1, Y1, Z1, X2, Y2, Z2, SpringLen : SINGLE);

begin

  if Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1)) > 1E-6 then
  begin

  FX1 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (X2-X1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FY1 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (Y2-Y1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FZ1 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (Z2-Z1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FX2 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (X1-X2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FY2 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (Y1-Y2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FZ2 := SpringLen*TanH( Ln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1))/SpringLen)/Ln(2) )*( (Z1-Z2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

//  Writeln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1)));

  end;

end;

procedure CalcMuscleForces(X1, Y1, Z1, X2, Y2, Z2, MusclePower : SINGLE);

begin

  if Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1)) > 1E-6 then
  begin

  FX1 := MusclePower*( (X2-X1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FY1 := MusclePower*( (Y2-Y1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FZ1 := MusclePower*( (Z2-Z1)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FX2 := MusclePower*( (X1-X2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FY2 := MusclePower*( (Y1-Y2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

  FZ2 := MusclePower*( (Z1-Z2)/Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+sqr(Z2-Z1)) );

//  Writeln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1)));

  end
  else
  begin

  FX1 := 0;

  FY1 := 0;

  FZ1 := 0;

  FX2 := 0;

  FY2 := 0;

  FZ2 := 0;

//  Writeln(Sqrt(Sqr(X2-X1)+Sqr(Y2-Y1)+Sqr(Z2-Z1)));

  end;

end;

constructor TDemoSceneCatapult.Create(const AKraftPhysics:TKraft);
const //Count={4}6;
      Spacing=0.125;
var Index:longint;
//    RigidBodyBox:{array[0..Count-1] of} TKraftRigidBody;
    ShapeBox:TKraftShapeBox;
//    RigidBodyCapsule:array[0..Count-1] of TKraftRigidBody;
    ShapeCapsule:TKraftShapeCapsule;
//    TimeVar : TKraftTimeStep;
//    TimerVar : TKraftHighResolutionTimer;

begin
 inherited Create(AKraftPhysics);

// TimerVar.Create;

// TimerVar.SetFrameRate(60);

// Writeln('timestep: ', TimeVar.DeltaTime);

// Delay(8000);

// SetFrameRate(100);

 AKraftPhysics.SetFrequency({250}150);

// AKraftPhysics.GravityScale := 10;

 RigidBodyFloor:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyFloor.SetRigidBodyType(krbtSTATIC);
 ShapeFloorPlane:=TKraftShapePlane.Create(KraftPhysics,RigidBodyFloor,Plane(Vector3Norm(Vector3(0.0,1.0,0.0)),0.0));
 ShapeFloorPlane.Restitution:=0.01;
 RigidBodyFloor.Finish;
 RigidBodyFloor.SetWorldTransformation(Matrix4x4Translate(0.0,0.0,0.0));
 RigidBodyFloor.CollisionGroups:=[0];

 RigidBodyCapsule[0]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[0].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[0],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[0].Finish;
 RigidBodyCapsule[0].CollisionGroups:=[0];
 RigidBodyCapsule[0].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}{0.75}0.85,{0.5}1.0,1.25)));
// Writeln('mass=', RigidBodyCapsule[0].Mass);
 RigidBodyCapsule[0].ForcedMass := RigidBodyCapsule[0].Mass*0.1;
// Writeln('mass=', RigidBodyCapsule[0].ForcedMass);

// Delay(8000);

 RigidBodyCapsule[1]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[1].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[1],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[1].Finish;
 RigidBodyCapsule[1].CollisionGroups:=[0];
 RigidBodyCapsule[1].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}1.7,{0.5}1.0,1.25)));
 RigidBodyCapsule[1].ForcedMass := RigidBodyCapsule[1].Mass*0.1;

 RigidBodyCapsule[2]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[2].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[2],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[2].Finish;
 RigidBodyCapsule[2].CollisionGroups:=[0];
 RigidBodyCapsule[2].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{0.75}0.85,{0.5}1.0,1.25)));
 RigidBodyCapsule[2].ForcedMass := RigidBodyCapsule[2].Mass*0.1;

 RigidBodyCapsule[3]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[3].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[3],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[3].Finish;
 RigidBodyCapsule[3].CollisionGroups:=[0];
 RigidBodyCapsule[3].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{1.5}1.7,{0.5}1.0,1.25)));
 RigidBodyCapsule[3].ForcedMass := RigidBodyCapsule[3].Mass*0.1;

 RigidBodyCapsule[4]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[4].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[4],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[4].Finish;
 RigidBodyCapsule[4].CollisionGroups:=[0];
 RigidBodyCapsule[4].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}{0.75}0.85,{0.5}1.0,0.75)));
 RigidBodyCapsule[4].ForcedMass := RigidBodyCapsule[4].Mass*0.1;

 RigidBodyCapsule[5]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[5].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[5],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[5].Finish;
 RigidBodyCapsule[5].CollisionGroups:=[0];
 RigidBodyCapsule[5].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}{1.5}1.7,{0.5}1.0,0.75)));
 RigidBodyCapsule[5].ForcedMass := RigidBodyCapsule[5].Mass*0.1;

 RigidBodyCapsule[6]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[6].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[6],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[6].Finish;
 RigidBodyCapsule[6].CollisionGroups:=[0];
 RigidBodyCapsule[6].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{0.75}0.85,{0.5}1.0,0.75)));
 RigidBodyCapsule[6].ForcedMass := RigidBodyCapsule[6].Mass*0.1;

 RigidBodyCapsule[7]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[7].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[7],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[7].Finish;
 RigidBodyCapsule[7].CollisionGroups:=[0];
 RigidBodyCapsule[7].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{1.5}1.7,{0.5}1.0,0.75)));
 RigidBodyCapsule[7].ForcedMass := RigidBodyCapsule[7].Mass*0.1;

 RigidBodyCapsule[8]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[8].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[8],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[8].Finish;
 RigidBodyCapsule[8].CollisionGroups:=[0];
 RigidBodyCapsule[8].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}{0.75}0.85,{0.5}1.0,0.25)));
 RigidBodyCapsule[8].ForcedMass := RigidBodyCapsule[8].Mass*0.1;

 RigidBodyCapsule[9]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[9].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[9],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[9].Finish;
 RigidBodyCapsule[9].CollisionGroups:=[0];
 RigidBodyCapsule[9].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(0.5*pi),Matrix4x4Translate(-{1.0}{1.5}1.7,{0.5}1.0,0.25)));
 RigidBodyCapsule[9].ForcedMass := RigidBodyCapsule[9].Mass*0.1;

 RigidBodyCapsule[10]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[10].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[10],{0.25}0.04,0.5);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[10].Finish;
 RigidBodyCapsule[10].CollisionGroups:=[0];
 RigidBodyCapsule[10].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{0.75}0.85,{0.5}1.0,0.25)));
 RigidBodyCapsule[10].ForcedMass := RigidBodyCapsule[10].Mass*0.1;

 RigidBodyCapsule[11]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyCapsule[11].SetRigidBodyType(krbtDYNAMIC{STATIC});
 ShapeCapsule:=TKraftShapeCapsule.Create(KraftPhysics,RigidBodyCapsule[11],{0.25}0.04,1.0);
 ShapeCapsule.Restitution:=0.01;
 RigidBodyCapsule[11].Finish;
 RigidBodyCapsule[11].CollisionGroups:=[0];
 RigidBodyCapsule[11].SetWorldTransformation(Matrix4x4TermMul(Matrix4x4RotateZ(-0.5*pi),Matrix4x4Translate({1.0}{1.5}1.7,{0.5}1.0,0.25)));
 RigidBodyCapsule[11].ForcedMass := RigidBodyCapsule[11].Mass*0.1;

 RigidBodyBox[0]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyBox[0].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyBox[0],Vector3(0.5,0.1,0.85));
 ShapeBox.Restitution:=0.01;
//ShapeBox.Density:=100.0;
 RigidBodyBox[0].Finish;
 RigidBodyBox[0].SetWorldTransformation(Matrix4x4Translate(0.0,{1.0}1.00,0.75));
 RigidBodyBox[0].CollisionGroups:=[0];
 RigidBodyBox[0].ForcedMass := RigidBodyBox[0].Mass*0.1;

(* ... RigidBodyMassBox[0]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[0].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[0],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[0].Finish;
 RigidBodyMassBox[0].SetWorldTransformation(Matrix4x4Translate(-{1.0}0.5,{0.5}1.0,1.5));
 RigidBodyMassBox[0].CollisionGroups:=[0];
 RigidBodyMassBox[0].ForcedMass := RigidBodyMassBox[0].Mass*0.1;

 RigidBodyMassBox[1]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[1].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[1],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[1].Finish;
 RigidBodyMassBox[1].SetWorldTransformation(Matrix4x4Translate({1.0}0.5,{0.5}1.0,1.5));
 RigidBodyMassBox[1].CollisionGroups:=[0];
 RigidBodyMassBox[1].ForcedMass := RigidBodyMassBox[1].Mass*0.1;

 RigidBodyMassBox[2]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[2].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[2],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[2].Finish;
 RigidBodyMassBox[2].SetWorldTransformation(Matrix4x4Translate(-{1.0}0.5,{0.5}1.0,1.0));
 RigidBodyMassBox[2].CollisionGroups:=[0];
 RigidBodyMassBox[2].ForcedMass := RigidBodyMassBox[2].Mass*0.1;

 RigidBodyMassBox[3]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[3].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[3],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[3].Finish;
 RigidBodyMassBox[3].SetWorldTransformation(Matrix4x4Translate({1.0}0.5,{0.5}1.0,1.0));
 RigidBodyMassBox[3].CollisionGroups:=[0];
 RigidBodyMassBox[3].ForcedMass := RigidBodyMassBox[3].Mass*0.1;

 RigidBodyMassBox[4]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[4].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[4],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[4].Finish;
 RigidBodyMassBox[4].SetWorldTransformation(Matrix4x4Translate(-{1.0}0.5,{0.5}1.0,0.5));
 RigidBodyMassBox[4].CollisionGroups:=[0];
 RigidBodyMassBox[4].ForcedMass := RigidBodyMassBox[4].Mass*0.1;

 RigidBodyMassBox[5]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[5].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[5],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[5].Finish;
 RigidBodyMassBox[5].SetWorldTransformation(Matrix4x4Translate({1.0}0.5,{0.5}1.0,0.5));
 RigidBodyMassBox[5].CollisionGroups:=[0];
 RigidBodyMassBox[5].ForcedMass := RigidBodyMassBox[5].Mass*0.1;

 RigidBodyMassBox[6]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[6].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[6],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[6].Finish;
 RigidBodyMassBox[6].SetWorldTransformation(Matrix4x4Translate(-{1.0}0.5,{0.5}1.0,0.0));
 RigidBodyMassBox[6].CollisionGroups:=[0];
 RigidBodyMassBox[6].ForcedMass := RigidBodyMassBox[6].Mass*0.1;

 RigidBodyMassBox[7]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyMassBox[7].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyMassBox[7],Vector3(0.1,0.1,0.1));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyMassBox[7].Finish;
 RigidBodyMassBox[7].SetWorldTransformation(Matrix4x4Translate({1.0}0.5,{0.5}1.0,0.0));
 RigidBodyMassBox[7].CollisionGroups:=[0];
 RigidBodyMassBox[7].ForcedMass := RigidBodyMassBox[7].Mass*0.1;  ... *)

(*.. RigidBodyBox[1]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyBox[1].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyBox[1],Vector3(0.5,0.1,0.75));
 ShapeBox.Restitution:=0.3;
//ShapeBox.Density:=100.0;
 RigidBodyBox[1].Finish;
 RigidBodyBox[1].SetWorldTransformation(Matrix4x4Translate(0.0,{1.0}1.20,0.75));
 RigidBodyBox[1].CollisionGroups:=[0]; *)

(* RigidBodyBox[1]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyBox[1].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyBox[1],Vector3(0.5,0.5,0.5));
 ShapeBox.Restitution:=0.3;
 ShapeBox.Density:=50.0;
 RigidBodyBox[1].Finish;
 RigidBodyBox[1].SetWorldTransformation(Matrix4x4Translate(1.5,8.0,0.0));
 RigidBodyBox[1].CollisionGroups:=[0];

 RigidBodyBox[2]:=TKraftRigidBody.Create(KraftPhysics);
 RigidBodyBox[2].SetRigidBodyType(krbtDYNAMIC);
 ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyBox[2],Vector3(0.25,0.25,0.25));
 ShapeBox.Restitution:=0.3;
 ShapeBox.Density:=50.0;
 RigidBodyBox[2].Finish;
 RigidBodyBox[2].SetWorldTransformation(Matrix4x4Translate(-1.5,1.25,0.0));
 RigidBodyBox[2].CollisionGroups:=[0]; *)

// TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyBox[0],RigidBodyBox[1],Vector3({0.0}{-0.5}{-1.0}0.0,{0.5}{0.2}1.1,{0.0}1.25),Vector3(1.0,{0.0}0.0,{1.0}{0.4}0.0),false,false,-0.25,0.25,0.0,0.0,false);

// TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyBox[0],RigidBodyBox[1],Vector3({0.0}{-0.5}{-1.0}0.0,{0.5}{0.2}1.1,{0.0}0.25),Vector3(1.0,{0.0}0.0,{1.0}{0.4}0.0),false,false,-0.25,0.25,0.0,0.0,false);

 Joint[0] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[0],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.55,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[0].EnableMotor(False);

 Joint[0].EnableLimit(False);

// Joint1.SetWorldRotationAxis(Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0));

 //Joint1.SetMaximalMotorTorque(-100.0);

// Joint1.InitializeConstraintsAndWarmStart;

(* Joint1.EnableLimit(True);

 Joint1.EnableMotor(True);

 Joint1.SetMotorSpeed(2000.0);

 Joint1.SetMinimumAngleLimit(-1000);

 Joint1.SetMaximumAngleLimit(1000); *)

// TKraftConstraintJointHinge.EnableLimit(True);

// TKraftConstraintJointHinge.EnableMotor(True);

 Joint[1] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[1],RigidBodyCapsule[0],Vector3({0.0}{-0.5}{-1.0}-1.1,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[1].EnableMotor(False);

 Joint[1].EnableLimit(False);

 Joint[2] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[2],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.55,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[2].EnableMotor(False);

 Joint[2].EnableLimit(False);

 Joint[3] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[3],RigidBodyCapsule[2],Vector3({0.0}{-0.5}{-1.0}1.1,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[3].EnableMotor(False);

 Joint[3].EnableLimit(False);

 Joint[4] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[4],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.55,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[4].EnableMotor(False);

 Joint[4].EnableLimit(False);

 Joint[5] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[5],RigidBodyCapsule[4],Vector3({0.0}{-0.5}{-1.0}-1.1,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[5].EnableMotor(False);

 Joint[5].EnableLimit(False);

 Joint[6] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[6],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.55,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[6].EnableMotor(False);

 Joint[6].EnableLimit(False);

 Joint[7] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[7],RigidBodyCapsule[6],Vector3({0.0}{-0.5}{-1.0}1.1,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[7].EnableMotor(False);

 Joint[7].EnableLimit(False);

 Joint[8] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[8],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.55,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[8].EnableMotor(False);

 Joint[8].EnableLimit(False);

 Joint[9] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[9],RigidBodyCapsule[8],Vector3({0.0}{-0.5}{-1.0}-1.1,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[9].EnableMotor(False);

 Joint[9].EnableLimit(False);

 Joint[10] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[10],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.55,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true,false,-0.15*Pi,0.15*Pi,0.0,-1.0,true);

 Joint[10].EnableMotor(False);

 Joint[10].EnableLimit(False);

 Joint[11] := TKraftConstraintJointHinge.Create(KraftPhysics,RigidBodyCapsule[11],RigidBodyCapsule[10],Vector3({0.0}{-0.5}{-1.0}1.1,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true,false,0.3*Pi,0.5*Pi,0.0,-1.0,true);

 Joint[11].EnableMotor(False);

 Joint[11].EnableLimit(False);

//

(*TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[0],RigidBodyMassBox[0],Vector3(-0.5,0,0),Vector3(-0.5,0,0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[1],RigidBodyCapsule[0],Vector3(0,0,0),Vector3(0,0,0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[2],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[3],RigidBodyCapsule[2],Vector3({0.0}{-0.5}{-1.0}1.0,{0.5}{0.2}1.0,{0.0}1.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[4],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[5],RigidBodyCapsule[4],Vector3({0.0}{-0.5}{-1.0}-1.0,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[6],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[7],RigidBodyCapsule[6],Vector3({0.0}{-0.5}{-1.0}1.0,{0.5}{0.2}1.0,{0.0}0.75),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[8],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[9],RigidBodyCapsule[8],Vector3({0.0}{-0.5}{-1.0}-1.0,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[10],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}1.0,{1.0}{0.4}0.0),true);

TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyCapsule[11],RigidBodyCapsule[10],Vector3({0.0}{-0.5}{-1.0}1.0,{0.5}{0.2}1.0,{0.0}0.25),Vector3(0.0,{0.0}0.0,{1.0}{0.4}1.0),true); *)

 //..

(*  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[0],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}1.5),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[1],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}1.5),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[2],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}1.0),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[3],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}1.0),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[4],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}0.5),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[5],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}0.5),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[6],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}-0.5,{0.5}{0.2}1.0,{0.0}0.0),false);

  TKraftConstraintJointFixed.Create(KraftPhysics,RigidBodyMassBox[7],RigidBodyBox[0],Vector3({0.0}{-0.5}{-1.0}0.5,{0.5}{0.2}1.0,{0.0}0.0),false); *)

  //

(*  RigidBodyMassBox[0].LinearVelocityDamp := 30;
  RigidBodyMassBox[0].GravityScale := 25;
  RigidBodyMassBox[1].LinearVelocityDamp := 30;
  RigidBodyMassBox[1].GravityScale := 25;
  RigidBodyMassBox[2].LinearVelocityDamp := 30;
  RigidBodyMassBox[2].GravityScale := 25;
  RigidBodyMassBox[3].LinearVelocityDamp := 30;
  RigidBodyMassBox[3].GravityScale := 25;
  RigidBodyMassBox[4].LinearVelocityDamp := 30;
  RigidBodyMassBox[4].GravityScale := 25;
  RigidBodyMassBox[5].LinearVelocityDamp := 30;
  RigidBodyMassBox[5].GravityScale := 25;
  RigidBodyMassBox[6].LinearVelocityDamp := 30;
  RigidBodyMassBox[6].GravityScale := 25;
  RigidBodyMassBox[7].LinearVelocityDamp := {50}30;
  RigidBodyMassBox[7].GravityScale := 25;

  RigidBodyMassBox[0].AngularVelocityDamp := 15;
  RigidBodyMassBox[1].AngularVelocityDamp := 15;
  RigidBodyMassBox[2].AngularVelocityDamp := 15;
  RigidBodyMassBox[3].AngularVelocityDamp := 15;
  RigidBodyMassBox[4].AngularVelocityDamp := 15;
  RigidBodyMassBox[5].AngularVelocityDamp := 15;
  RigidBodyMassBox[6].AngularVelocityDamp := 15;
  RigidBodyMassBox[7].AngularVelocityDamp := {50}{3}{12}{10}15; *)

  RigidBodyCapsule[0].LinearVelocityDamp := 120;
  RigidBodyCapsule[0].GravityScale := 10;
  RigidBodyCapsule[1].LinearVelocityDamp := 120;
  RigidBodyCapsule[1].GravityScale := 10;
  RigidBodyCapsule[2].LinearVelocityDamp := 120;
  RigidBodyCapsule[2].GravityScale := 10;
  RigidBodyCapsule[3].LinearVelocityDamp := 120;
  RigidBodyCapsule[3].GravityScale := 10;
  RigidBodyCapsule[4].LinearVelocityDamp := 120;
  RigidBodyCapsule[4].GravityScale := 10;
  RigidBodyCapsule[5].LinearVelocityDamp := 120;
  RigidBodyCapsule[5].GravityScale := 10;
  RigidBodyCapsule[6].LinearVelocityDamp := 120;
  RigidBodyCapsule[6].GravityScale := 10;
  RigidBodyCapsule[7].LinearVelocityDamp := 120;
  RigidBodyCapsule[7].GravityScale := 10;
  RigidBodyCapsule[8].LinearVelocityDamp := 120;
  RigidBodyCapsule[8].GravityScale := 10;
  RigidBodyCapsule[9].LinearVelocityDamp := 120;
  RigidBodyCapsule[9].GravityScale := 10;
  RigidBodyCapsule[10].LinearVelocityDamp := 120;
  RigidBodyCapsule[10].GravityScale := 10;
  RigidBodyCapsule[11].LinearVelocityDamp := {20}120;
  RigidBodyCapsule[11].GravityScale := 10;

  RigidBodyCapsule[0].AngularVelocityDamp := 60;
  RigidBodyCapsule[1].AngularVelocityDamp := 60;
  RigidBodyCapsule[2].AngularVelocityDamp := 60;
  RigidBodyCapsule[3].AngularVelocityDamp := 60;
  RigidBodyCapsule[4].AngularVelocityDamp := 60;
  RigidBodyCapsule[5].AngularVelocityDamp := 60;
  RigidBodyCapsule[6].AngularVelocityDamp := 60;
  RigidBodyCapsule[7].AngularVelocityDamp := 60;
  RigidBodyCapsule[8].AngularVelocityDamp := 60;
  RigidBodyCapsule[9].AngularVelocityDamp := 60;
  RigidBodyCapsule[10].AngularVelocityDamp := 60;
  RigidBodyCapsule[11].AngularVelocityDamp := {20}{2}{5}{4}60;

  RigidBodyBox[0].LinearVelocityDamp := 180;
  RigidBodyBox[0].AngularVelocityDamp := {50}{12}{10}90;
  RigidBodyBox[0].GravityScale := 200;

{for Index:=0 to Count-1 do begin
  RigidBodyBox[Index]:=TKraftRigidBody.Create(KraftPhysics);
  if (Index=0) or (Index=(Count-1)) then begin
   RigidBodyBox[Index].SetRigidBodyType(krbtSTATIC);
  end else begin
   RigidBodyBox[Index].SetRigidBodyType(krbtDYNAMIC);
  end;
  ShapeBox:=TKraftShapeBox.Create(KraftPhysics,RigidBodyBox[Index],Vector3(0.5,0.1,1.0));
  ShapeBox.Restitution:=0.3;
  ShapeBox.Density:=100.0;
  RigidBodyBox[Index].Finish;
  RigidBodyBox[Index].SetWorldTransformation(Matrix4x4Translate(((ShapeBox.Extents.x*2.0)+Spacing)*(Index-((Count-1)*0.5)),5.0,-4.0));
  RigidBodyBox[Index].CollisionGroups:=[0];
 end;}

{
 for Index:=1 to Count-1 do begin
  TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyBox[Index-1],RigidBodyBox[Index],Vector3(ShapeBox.Extents.x+(Spacing*0.5),0.0,-ShapeBox.Extents.z),Vector3(-(ShapeBox.Extents.x+(Spacing*0.5)),0.0,-ShapeBox.Extents.z));
  TKraftConstraintJointBallSocket.Create(KraftPhysics,RigidBodyBox[Index-1],RigidBodyBox[Index],Vector3(ShapeBox.Extents.x+(Spacing*0.5),0.0,ShapeBox.Extents.z),Vector3(-(ShapeBox.Extents.x+(Spacing*0.5)),0.0,ShapeBox.Extents.z));
 end;
 {}

end;

destructor TDemoSceneCatapult.Destroy;
begin
 inherited Destroy;
end;

procedure TDemoSceneCatapult.Step(const DeltaTime:double);

var quaternion_xyz{, center_xyz2} : {TKraftRawVector3}TKraftQuaternion;
    center_xyz, center_xyz2 : TKraftRawVector3;
    point_xyz, vel_xyz, vel2_xyz : TKraftVector3;
    FX_new, FZ_new, FY_new : SINGLE;

begin

(*   RigidBodyCapsule[0].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[1].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);

   RigidBodyCapsule[2].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[3].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);

   RigidBodyCapsule[4].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[5].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);

   RigidBodyCapsule[6].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[7].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);

   RigidBodyCapsule[8].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[9].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);

   RigidBodyCapsule[10].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);
   RigidBodyCapsule[11].AddBodyForce{AngularVelocity}(Vector3(Random*10,Random*10,Random*10),kfmAcceleration);*)

(*   quaternion_xyz := RigidBodyBox[0].Sweep.q;

//   Writeln(center_xyz.x, center_xyz.y);

   if Sqrt(Sqr(Sin(quaternion_xyz.y))+Sqr(Cos(quaternion_xyz.y))) > 1E-4 then
     FX_new := Sqrt(1-Sqr(Sin(quaternion_xyz.x*Pi)))*(Sin(quaternion_xyz.y*Pi)/Sqrt(Sqr(Sin(quaternion_xyz.y*Pi))+Sqr(Cos(quaternion_xyz.y*Pi))))
   else
     FX_new := Sqrt(1-Sqr(Sin(quaternion_xyz.x*Pi)))*(Sin(quaternion_xyz.y*Pi)/1E-4);

   if Sqrt(Sqr(Sin(quaternion_xyz.y))+Sqr(Cos(quaternion_xyz.y))) > 1E-4 then
     FZ_new := Sqrt(1-Sqr(Sin(quaternion_xyz.x*Pi)))*(Cos(quaternion_xyz.y*Pi)/Sqrt(Sqr(Sin(quaternion_xyz.y*Pi))+Sqr(Cos(quaternion_xyz.y*Pi))))
   else
     FZ_new := Sqrt(1-Sqr(Sin(quaternion_xyz.x*Pi)))*(Cos(quaternion_xyz.y*Pi)/1E-4);

   FY_new := Sin(quaternion_xyz.x*Pi); .. *)

//   Writeln(FX_new, ' - ', FY_new, ' - ', FZ_new);

//   center_xyz2 := RigidBodyBox[1].Sweep.c.RawVector;

//   FindNormal();

//  Sin(Iteration*2*Pi*0.004)

//.  .   RigidBodyCapsule[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

//..   RigidBodyCapsule[2].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-500*FX_new,(Sin(Iteration*2*Pi*0.004))*-500*FY_new,(Sin(Iteration*2*Pi*0.004))*-500*FZ_new),kfmAcceleration);

//   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-500*FX_new,(Sin(Iteration*2*Pi*0.004))*-500*FY_new,(Sin(Iteration*2*Pi*0.004))*-500*FZ_new),kfmAcceleration);

(*   RigidBodyCapsule[2].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

//   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-50*FX_new,(Sin(Iteration*2*Pi*0.004))*-50*FY_new,(Sin(Iteration*2*Pi*0.004))*-50*FZ_new),kfmAcceleration);

   RigidBodyCapsule[4].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

//   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-50*FX_new,(Sin(Iteration*2*Pi*0.004))*-50*FY_new,(Sin(Iteration*2*Pi*0.004))*-50*FZ_new),kfmAcceleration);

   RigidBodyCapsule[6].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

//   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-50*FX_new,(Sin(Iteration*2*Pi*0.004))*-50*FY_new,(Sin(Iteration*2*Pi*0.004))*-50*FZ_new),kfmAcceleration);

   RigidBodyCapsule[8].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

//   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-50*FX_new,(Sin(Iteration*2*Pi*0.004))*-50*FY_new,(Sin(Iteration*2*Pi*0.004))*-50*FZ_new),kfmAcceleration);

   RigidBodyCapsule[10].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

   RigidBodyBox[0].AddBodyForce{AngularVelocity}(Vector3((Sin(Iteration*2*Pi*0.004))*-50*3*FX_new,(Sin(Iteration*2*Pi*0.004))*-50*3*FY_new,(Sin(Iteration*2*Pi*0.004))*-50*3*FZ_new),kfmAcceleration); *)

//   RigidBodyCapsule[0].AddBodyAngularVelocity(Vector3((Sin(Iteration*2*Pi*0.004))*50*FX_new,(Sin(Iteration*2*Pi*0.004))*50*FY_new,(Sin(Iteration*2*Pi*0.004))*50*FZ_new),kfmAcceleration);

// ..
(* ...   center_xyz := RigidBodyCapsule[0].Sweep.c.RawVector;
   center_xyz2 := RigidBodyMassBox[0].Sweep.c.RawVector;

   CalcSpringForces(center_xyz.x, center_xyz.y, center_xyz.z, center_xyz2.x, center_xyz2.y, center_xyz2.z, {1}0.36);

   RigidBodyCapsule[0].AddBodyForce{AngularVelocity}(Vector3(FX1*260,FY1*260,FZ1*260),kfmAcceleration);
   RigidBodyMassBox[0].AddBodyForce{AngularVelocity}(Vector3(FX2*260,FY2*260,FZ2*260),kfmAcceleration);

   center_xyz := RigidBodyCapsule[0].Sweep.c.RawVector;
   center_xyz2 := RigidBodyCapsule[1].Sweep.c.RawVector;

   CalcSpringForces(center_xyz.x, center_xyz.y, center_xyz.z, center_xyz2.x, center_xyz2.y, center_xyz2.z, {1}0.29);

   RigidBodyCapsule[0].AddBodyForce{AngularVelocity}(Vector3(FX1*260,FY1*260,FZ1*260),kfmAcceleration);
   RigidBodyCapsule[1].AddBodyForce{AngularVelocity}(Vector3(FX2*260,FY2*260,FZ2*260),kfmAcceleration);

   center_xyz := RigidBodyCapsule[2].Sweep.c.RawVector;
   center_xyz2 := RigidBodyMassBox[1].Sweep.c.RawVector;

   CalcSpringForces(center_xyz.x, center_xyz.y, center_xyz.z, center_xyz2.x, center_xyz2.y, center_xyz2.z, {1}0.36);

   RigidBodyCapsule[2].AddBodyForce{AngularVelocity}(Vector3(FX1*260,FY1*260,FZ1*260),kfmAcceleration);
   RigidBodyMassBox[1].AddBodyForce{AngularVelocity}(Vector3(FX2*260,FY2*260,FZ2*260),kfmAcceleration);

   center_xyz := RigidBodyCapsule[2].Sweep.c.RawVector;
   center_xyz2 := RigidBodyCapsule[3].Sweep.c.RawVector;

   CalcSpringForces(center_xyz.x, center_xyz.y, center_xyz.z, center_xyz2.x, center_xyz2.y, center_xyz2.z, {1}0.29);

   RigidBodyCapsule[2].AddBodyForce{AngularVelocity}(Vector3(FX1*260,FY1*260,FZ1*260),kfmAcceleration);
   RigidBodyCapsule[3].AddBodyForce{AngularVelocity}(Vector3(FX2*260,FY2*260,FZ2*260),kfmAcceleration);  ... *)

   // -

(*  Joint1.EnableLimit(True);

  Joint1.EnableMotor(True);

  Joint1.SetMotorSpeed(2000.0);

  Joint1.SetMinimumAngleLimit(-1);

  Joint1.SetMaximumAngleLimit(1);

  Joint1.SetMaximalMotorTorque(2000); *)

// Joint1.AddTorque(Vector3(FX1*260,FY1*260,FZ1*260),kfmAcceleration);

  Joint[0].EnableMotor(True);

  Joint[0].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[0].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[0].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0));

  Joint[0].SetMaximalMotorTorque({-100.0}-{Abs}(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[0].SetMinimumAngleLimit(-0.15*Pi);

  Joint[0].SetMaximumAngleLimit(0.15*Pi);

(*  if Sign(FMotor) >= 0 then
  begin

  Joint[0].SetMinimumAngleLimit(-0.15*Pi);

  Joint[0].SetMaximumAngleLimit(0.15*Pi);


  end
  else
  begin

  Joint[0].SetMinimumAngleLimit(0.15*Pi);

  Joint[0].SetMaximumAngleLimit(-0.15*Pi);


  end; *)

  Joint[1].EnableMotor(True);

  Joint[1].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[1].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[1].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[1].SetMaximalMotorTorque({-100.0}-{Abs}(FMotor));

  Joint[1].SetMinimumAngleLimit({(0.5*Pi-0.5*Pi)}-0.2*Pi);

  Joint[1].SetMaximumAngleLimit({(0.8*Pi-0.5*Pi)}0.0*Pi);

(*  if Sign(FMotor) >= 0 then
  begin

  Joint[1].SetMinimumAngleLimit(0.3*Pi);

  Joint[1].SetMaximumAngleLimit(0.5*Pi);


  end
  else
  begin

  Joint[1].SetMinimumAngleLimit(-0.3*Pi);

  Joint[1].SetMaximumAngleLimit(-0.5*Pi);


  end; *)

//

  Joint[2].EnableMotor(True);

//  Joint[2].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[2].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[2].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}-1.0,{1.0}{0.4}0.0));

  Joint[2].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[3].EnableMotor(True);

  Joint[3].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[3].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[3].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}-1.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[3].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//
  Joint[4].EnableMotor(True);

//  Joint[4].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[4].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[4].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}-1.0,{1.0}{0.4}0.0));

  Joint[4].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[5].EnableMotor(True);

//  Joint[5].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[5].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[5].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}-1.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[5].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//

    Joint[6].EnableMotor(True);

//  Joint[6].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[6].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[6].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}-1.0,{1.0}{0.4}0.0));

  Joint[6].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[7].EnableMotor(True);

//  Joint[7].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[7].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[7].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}-1.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[7].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//

  Joint[8].EnableMotor(True);

//  Joint[8].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[8].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[8].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}-1.0,{1.0}{0.4}0.0));

  Joint[8].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[9].EnableMotor(True);

//  Joint[9].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[9].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[9].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}-1.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[9].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//

  Joint[10].EnableMotor(True);

//  Joint[10].EnableLimit(True);

  FMotor := Sin((Iteration/300)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[10].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}1.0,{1.0}{0.4}0.0))
  else
    Joint[10].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}-1.0,{1.0}{0.4}0.0));

  Joint[10].SetMaximalMotorTorque({-100.0}-Abs(FMotor));

//  Joint[0].SetMotorSpeed({-100.0}(FMotor));

  Joint[11].EnableMotor(True);

//  Joint[11].EnableLimit(True);

  FMotor := Sin((Iteration/140)*2*Pi) * {12}50;

  if Sign(FMotor) >= 0 then
    Joint[11].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}1.0))
  else
    Joint[11].SetWorldRotationAxis(Vector3(0.0,{0.0}{1.0}0.0,{1.0}{0.4}-1.0));

//  Joint[1].SetMotorSpeed({-100.0}(FMotor));

  Joint[11].SetMaximalMotorTorque({-100.0}-Abs(FMotor));


   // -

// ..

   Inc(Iteration);

//   RigidBodyBox[0].WorldTransform;

end;

initialization
 RegisterDemoScene('Catapult',TDemoSceneCatapult);
end.
