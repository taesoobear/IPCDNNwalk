#include "stdafx.h"
#include "../BaseLib/utility/util.h"
#include "../BaseLib/utility/configtable.h"
#include "../BaseLib/utility/TextFile.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/Operator.h"
#include "./viewpoint.h"


#include <math.h>

#define MAX_UP_ANGLE	((89.999)*(M_PI/180.f))	// 90 degree
#define MAX_DOWN_ANGLE	((-89.999)*(M_PI/180.f)) // -90 degree

inline vector3 defaultViewDir(vector3 const& vup)
{
	if (isSimilar(vup.y, 1))
		return vector3(0,0,-1);
	return vector3(0,1,0);
}

//-----------------------------------------------------------------------------
// Name:
// Desc:
//-----------------------------------------------------------------------------


void Viewpoint::ReadViewPoint(FILE *fpFile)
{
	char *token;
	//view point position
	VERIFY(token = GetToken(fpFile));
	m_vecVPos.x = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVPos.y = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVPos.z = (m_real)atof(token);

	// view at
	VERIFY(token = GetToken(fpFile));
	m_vecVAt.x = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVAt.y = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVAt.z = (m_real)atof(token);

	// view up vector

	VERIFY(token = GetToken(fpFile));
	m_vecVUp.x = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVUp.y = (m_real)atof(token);
	VERIFY(token = GetToken(fpFile));
	m_vecVUp.z = (m_real)atof(token);

	VERIFY(token = GetToken(fpFile));
	m_fDesiredZoom= (m_real)atof(token);

	CalcHAngle();
	CalcVAngle();
	CalcDepth();
}

void Viewpoint::ReadViewPoint(char *fileName)
{
	char *token;
	CTextFile test;
	test.OpenReadFile(fileName);

	//view point position
	VERIFY(token = test.GetToken());
	m_vecVPos.x = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVPos.y = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVPos.z = (m_real)atof(token);

	// view at
	VERIFY(token = test.GetToken());
	m_vecVAt.x = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVAt.y = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVAt.z = (m_real)atof(token);

	// view up vector

	VERIFY(token = test.GetToken());
	m_vecVUp.x = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVUp.y = (m_real)atof(token);
	VERIFY(token = test.GetToken());
	m_vecVUp.z = (m_real)atof(token);

	VERIFY(token = test.GetToken());
	m_fDesiredZoom= (m_real)atof(token);

	CalcHAngle();
	CalcVAngle();
	CalcDepth();
}


void Viewpoint::interpolate(m_real t, const Viewpoint & a, const Viewpoint & b)
{
	m_vecVAt.interpolate(t, a.m_vecVAt, b.m_vecVAt);
	m_fDesiredZoom=sop::interpolate(t, a.m_fDesiredZoom, b.m_fDesiredZoom);
	m_fVAngle=sop::interpolate(t, a.m_fVAngle, b.m_fVAngle);

	quater qa(a.m_fHAngle, vector3(0,1,0));
	quater qb(b.m_fHAngle, vector3(0,1,0));
	qb.align(qa);
	quater q;
	q.interpolate(t, qa, qb);

	m_fHAngle=q.rotationAngleAboutAxis(vector3(0,1,0));
	m_fDepth=sop::interpolate(t, a.m_fDepth, b.m_fDepth);
	UpdateVPosFromVHD();
}

Viewpoint& Viewpoint::operator=(const Viewpoint& other)
{
	m_vecVPos=other.m_vecVPos;
	m_vecVAt=other.m_vecVAt;
	m_vecVUp=other.m_vecVUp;
	m_fDesiredZoom= other.m_fDesiredZoom;

	CalcHAngle();
	CalcVAngle();
	CalcDepth();

	return *this;
}

void Viewpoint::WriteViewPoint(FILE *fpFile)
{
	// 시점 위치
	fprintf(fpFile, "#시점 위치\n");
	fprintf(fpFile, "%f, %f, %f\n"
		, m_vecVPos.x
		, m_vecVPos.y
		, m_vecVPos.z);

	// 시점 지향점
	fprintf(fpFile, "#시점 지향점.\n");
	fprintf(fpFile, "%f, %f, %f\n"
		, m_vecVAt.x
		, m_vecVAt.y
		, m_vecVAt.z);
	
	// VUP
	fprintf(fpFile, "# VUP.\n\n");
	fprintf(fpFile, "%f, %f, %f\n"
		, m_vecVUp.x
		, m_vecVUp.y
		, m_vecVUp.z);
	
	fprintf(fpFile,"#desired zoom\n%f\n",m_fDesiredZoom);

}

/*inline m_real ABS(float a) { return (a>0)? a: (a*-1); };
inline float MAX(float a,float b, float c) {
	if(a>b)
	{
		if(a>c) return a;
		else return c;
	}
	else
	{
		if(c>b) return c;
		else return b;
	}
}*/

/*
float Viewpoint::GetDifferenceDHV(const Viewpoint &anotherview)
{
	ASSERT(0);
	float trans_speed= 0;//g_ConfigTable.GetFloat("TRANS_SPEED");
	float angle_speed = 0;//g_ConfigTable.GetFloat("ANGLE_SPEED");
	D3DXVECTOR3 temp;
	D3DXVec3Subtract(&temp, &(m_vecVAt), &(anotherview.m_vecVAt));
	
	return ( MAX3(ABS(m_fDepth - anotherview.m_fDepth) / trans_speed ,
			(ABS(m_fVAngle - anotherview.m_fVAngle) + ABS(m_fHAngle - anotherview.m_fHAngle)) / angle_speed ,
			(D3DXVec3Length(&temp) / trans_speed))
			);
}*/

int Viewpoint::CalcDepth()
{
	vector3 vecDiff = m_vecVPos - m_vecVAt;

	m_fDepth = vecDiff .length();

	return 1;
}

int Viewpoint::CalcVAngle()
{
#if 1

	vector3 vup;
	vector3 left,viewDir;
	viewDir=m_vecVAt-m_vecVPos;
	viewDir.normalize();
	left.cross(m_vecVUp,viewDir);
	left.normalize();
	vup.cross(viewDir, left);
	vup.normalize();
	//printf("vup! %s\n", vup.output().ptr());
	quater diff;
	diff.setAxisRotation(left, m_vecVUp, vup);
	m_fVAngle=diff.rotationAngleAboutAxis(left);

#else
	// works only when Y up.

	//   case VAngle==RAD(90):
	//          vpos
	//           |   
	//     vup --+
	//   case VAngle==RAD(0):
	//          vat --- vpos
	//
	m_real fTan;

	m_real fXZ;

	fXZ = (m_real)sqrt((m_vecVPos.x - m_vecVAt.x)*(m_vecVPos.x - m_vecVAt.x)
		+ (m_vecVPos.z - m_vecVAt.z)*(m_vecVPos.z - m_vecVAt.z));
	fTan = (m_vecVPos.y - m_vecVAt.y)/fXZ;

	m_fVAngle = (m_real)atan(fTan);
#endif
	return 1;
}

int Viewpoint::CalcHAngle()
{
#if 1
	vector3 viewDir;
	viewDir=m_vecVAt-m_vecVPos;
	viewDir.normalize();
	quater diff;
	diff.setAxisRotation(m_vecVUp, viewDir, defaultViewDir(m_vecVUp));
	m_fHAngle=-diff.rotationAngleAboutAxis(m_vecVUp);
	if (m_fHAngle<0 ) m_fHAngle+=2.0*M_PI;
	//printf("h: %s %f\n", left.output().ptr(), m_fHAngle);
#else
	// works only for Yup
	m_real fTan;

	if(!IsZero(m_vecVPos.z - m_vecVAt.z)){
		fTan = (m_real)fabs((m_vecVPos.x - m_vecVAt.x)/(m_vecVPos.z - m_vecVAt.z));
		m_fHAngle = (m_real)atan(fTan);
	}
	else 
		m_fHAngle = TO_RADIAN(90.f);
		

	if(m_vecVPos.z - m_vecVAt.z > 0){
		if(m_vecVPos.x - m_vecVAt.x < 0){
			m_fHAngle = TO_RADIAN(360.f) - m_fHAngle;
		}
	}
	else{
		if(m_vecVPos.x - m_vecVAt.x < 0){
			m_fHAngle += TO_RADIAN(180.f);
		}
		else 
			m_fHAngle = TO_RADIAN(180.f) - m_fHAngle;
	}
	printf("h! %f\n", m_fHAngle);
#endif

	return 1;
}
int Viewpoint::UpdateVPosFromVHD()
{
	vector3 vecToPos;

#if 1
	quater diffH, diffV;
	diffH.setRotation(m_vecVUp, m_fHAngle);
	
	vector3 vup;
	vup.rotate(diffV,m_vecVUp);
	vecToPos=diffH*defaultViewDir(m_vecVUp);

	vector3 left;
	left.cross(m_vecVUp, vecToPos);

	diffV.setRotation(left, m_fVAngle);
	vecToPos=(diffV*vecToPos)*-1;
	//printf("vecToPos: %s\n", vecToPos.output().ptr());
#else
	// works only when Y up.
	vecToPos.y = (m_real)sin(m_fVAngle);
	vecToPos.z = (m_real)cos(m_fVAngle)*(m_real)cos(m_fHAngle);
	vecToPos.x = (m_real)cos(m_fVAngle)*(m_real)sin(m_fHAngle);
	//printf("vecToPos! %s\n", vecToPos.output().ptr());
#endif

	m_vecVPos = vecToPos*m_fDepth + m_vecVAt;
	//printf("m_vecVPos: %s\n", m_vecVPos.output().ptr());
	return 1;
}

int Viewpoint::GetViewMatrix(matrix4& matView)
{
	matView.lookAtRH(m_vecVPos, m_vecVAt, m_vecVUp);
	return 1;
}

int Viewpoint::SetViewMatrix(matrix4 const& matView)
{
	matView.decomposeRH(m_vecVPos, m_vecVAt, m_vecVUp);
	return 1;
}

int Viewpoint::ZoomIn(m_real ZoomAmount)
{
	m_fDepth-=ZoomAmount;
	if(m_fDepth<0 )
	{
		m_fDepth+=ZoomAmount;
		return 0;
	}
	UpdateVPosFromVHD();
	return 1;
}
int Viewpoint::ZoomOut(m_real ZoomAmount)
{
	ZoomIn(-ZoomAmount);
	return 1;
}


int Viewpoint::TurnRight(m_real radian)
{
	m_fHAngle+=radian;
	UpdateVPosFromVHD();
	if(m_fHAngle>2.f*3.1415f)
		m_fHAngle-=2.f*3.1415f;
	else if(m_fHAngle<0) 
		m_fHAngle+=2.f*3.1415f;
	return 1;
}

int Viewpoint::TurnLeft(m_real radian)
{
	TurnRight(-1*radian);
	return 1;
}

int Viewpoint::CheckConstraint()
{
	if(m_fVAngle>MAX_UP_ANGLE)
	{
		m_fVAngle=MAX_UP_ANGLE;
	}
	else if(m_fVAngle<MAX_DOWN_ANGLE)
	{
		m_fVAngle=MAX_DOWN_ANGLE;
	}
	if(m_fVAngle<m_fMinAngle)
	{
		m_fVAngle=m_fMinAngle;
	}

	UpdateVPosFromVHD();
	return 1;
}

int Viewpoint::TurnUp(m_real radian)
{
	m_fVAngle+=radian;
	if(m_fVAngle>MAX_UP_ANGLE)
	{
		m_fVAngle=MAX_UP_ANGLE;
	}
	else if(m_fVAngle<MAX_DOWN_ANGLE) 
	{	
		m_fVAngle=MAX_DOWN_ANGLE;
	}
	if(m_fVAngle<m_fMinAngle)
	{
		m_fVAngle=m_fMinAngle;
	}
	UpdateVPosFromVHD();

	return 1;
}

int Viewpoint::TurnDown(m_real radian)
{
	TurnUp(-1*radian);
	return 1;
}
int Viewpoint::UpdateVHD()
{
	CalcHAngle();
	CalcVAngle();
	CalcDepth();
	return 1;
}
int Viewpoint::PanDown(m_real pan_amt)
{
	PanUp(-pan_amt);
	return 1;
}
void Viewpoint::setZUp()
{
	m_vecVUp=vector3(0,0,1);
	//UpdateVHD();
	UpdateVPosFromVHD();
}
void Viewpoint::setYUp()
{
	m_vecVUp=vector3(0,1,0);
	//UpdateVHD();
	UpdateVPosFromVHD();
}
int Viewpoint::PanUp(m_real pan_amt)
{
	// 수직 위로 올라가는 버젼
	vector3 vup;
	vector3 left,viewDir;
	viewDir=m_vecVAt-m_vecVPos;
	left.cross(m_vecVUp,viewDir);
	vup.cross(left,viewDir);
	vup.normalize();
	vup*=pan_amt;
	m_vecVAt.add(vup);
	m_vecVPos.add(vup);
	return 1;
}
int Viewpoint::PanForward(m_real pan_amt)
{
	// 수평 위로 올라가는 버젼
	Msg::error("viewpoint error");
	/*
	D3DXVECTOR3 left,up,viewDir;
	D3DXVec3Subtract(&viewDir,&m_vecVAt,&m_vecVPos);
	D3DXVec3Cross(&left,&m_vecVUp,&viewDir);
	D3DXVec3Cross(&up,&left,&m_vecVUp);
	D3DXVec3Normalize(&up,&up);
	D3DXVec3Scale(&up,&up,pan_amt);
	D3DXVec3Add(&m_vecVAt,&m_vecVAt,&up);
	D3DXVec3Add(&m_vecVPos,&m_vecVPos,&up);*/
	return 1;
}
int Viewpoint::PanBackward(m_real pan_amt)
{
	PanForward(-1*pan_amt);
	return 1;
}

int Viewpoint::PanRight(m_real pan_amt)
{
	PanLeft(-pan_amt);
	return 1;
}
int Viewpoint::PanLeft(m_real pan_amt)
{
	vector3 left,viewDir;

	viewDir=m_vecVAt-m_vecVPos;
	left.cross(m_vecVUp,viewDir);
	left.normalize();
	left*=pan_amt;
	m_vecVAt+=left;
	m_vecVPos+=left;
	return 1;
}

int Viewpoint::CameraTurnDown(m_real radian)
{
	Msg::error("viewpoint error");
	/*
	D3DXVECTOR3 vpos=m_vecVPos;
	TurnUp(radian);
	D3DXVECTOR3 delta=m_vecVPos-vpos;
	m_vecVPos-=delta;
	m_vecVAt-=delta;
	UpdateVHD();
*/

	return 1;
}
int Viewpoint::CameraTurnUp(m_real radian)
{
	CameraTurnDown(-radian);
	return 1;
}
int Viewpoint::CameraTurnRight(m_real radian)
{
	CameraTurnLeft(-radian);
	return 1;
}
int Viewpoint::CameraTurnLeft(m_real radian)
{
	Msg::error("viewpoint error");
	/*
	D3DXVECTOR3 temp,temp2;
	D3DXMATRIX matRot;
	D3DXVec3Subtract(&temp,&m_vecVAt,&m_vecVPos);
	D3DXMatrixRotationAxis(&matRot,&m_vecVUp, radian);
	D3DXVec3TransformCoord(&temp2, &temp, &matRot);
	D3DXVec3Add(&m_vecVAt,&temp2,&m_vecVPos);
	UpdateVHD();*/
	return 1;
}

m_real Viewpoint::CalcNearMinY()
{
	//현재 view frustum에서 near plane의 가장 낮은 곳의 z좌표(근사값)
	m_real fNear=150.f;
	m_real dFov=40.f;
	m_real a;
	m_real b;
	b=TO_RADIAN(dFov/2.f);
	a=fNear/(m_real)cos(b);
/*#ifdef _DEBUG
	char sz[100];
	sprintf(sz,"b = %f a= %f ", b,a);
	strcat(g_szDebugText,sz);
#endif*/
	return m_vecVPos.y-a*(m_real)sin(b-m_fVAngle);

}

bool Viewpoint::HandleMouseMessages2( msgT uMsg, int iMouseX, int iMouseY)
{
	bool res=true;
	static bool m_bDrag=false;
	static int         downMouseX;      // Saved mouse position
    static int         downMouseY;
	static Viewpoint	downView;
	static int m_saveMsg=-1;
    // Current mouse position

	if(uMsg==LBUTTONDOWN || uMsg==RBUTTONDOWN || uMsg==MBUTTONDOWN)
	{
		downView=(*this);
		downMouseX = iMouseX;
        downMouseY = iMouseY;
		m_bDrag=true;
		m_saveMsg=uMsg;
	}
	
	if(m_bDrag)
	{
		int sx=iMouseX-downMouseX;
		int sy=iMouseY-downMouseY;
			
		if( m_saveMsg==LBUTTONDOWN)
		{
		//	this->operator =(downView);

			 // Scale to screen
			m_real x   = -(m_real)sx  / (m_iWidth/1.0);
			m_real y   =  (m_real)sy  / (m_iHeight/2.0);
			//x축,y축 회전
			TurnRight(x);
			TurnUp(y);
			CheckConstraint();
		}
		else if(m_saveMsg==RBUTTONDOWN)
		{
		//	this->operator =(downView);
			
			 // Scale to screen
			m_real x   = -(m_real)sx  / (m_iWidth/2.0);
			m_real y   =  (m_real)sy  / (m_iHeight/2.0);

			if(!bOrthographicMode)
			{
				x*=m_scale;
				y*=m_scale;
			}
			else
			{
				x*=(m_scale/m_zoom);
				y*=(m_scale/m_zoom);
			}
			//x축,y축 이동
			PanRight(x);
			PanDown(y);
			CheckConstraint();
		}
		else if(m_saveMsg==MBUTTONDOWN)
		{
			
			if(bOrthographicMode)
			{		
				m_real y   =  (m_real)sy  / (m_iHeight/2.0);
				//zoom in/out
				if(m_zoom<0.1)
				{
					m_zoom=0.1;
				}
				else if(m_zoom>5)
				{
					m_zoom=5;
				}
				else
				{
					m_zoom+=y*0.5;
					if(m_zoom<0.1)
					{
						m_zoom=0.1;
					}	
				}
			
				CheckConstraint();
			}
			else
			{
				//	this->operator =(downView);

				// Scale to screen
				m_real y   =  (m_real)sy  / (m_iHeight/2.0);
				y*=m_scale;
				ZoomOut(y*-1);
				CheckConstraint();
			}
		}

		downMouseX = iMouseX;
		downMouseY = iMouseY;
		
	}

	if(uMsg==LBUTTONUP || uMsg==RBUTTONUP || uMsg==MBUTTONUP)
	{
		m_bDrag=false;
	}
	
	return res;
}
bool Viewpoint::getOrthographicMode()
{
return bOrthographicMode;
}
void Viewpoint::setOrthographicMode(bool bOrtho)
{
	if(bOrtho)
	{
		if(bOrthographicMode==0)
		{
			bOrthographicMode=1;
			//m_fDepth=m_fDepth*10;
			UpdateVPosFromVHD();
		}
	}
	else
	{
		if(bOrthographicMode==1)
		{
			bOrthographicMode=0;
			UpdateVPosFromVHD();
		}
	}
}
m_real Viewpoint::getZoom(void)
{
	return m_zoom;
}
void Viewpoint::setZoom(m_real zoom)
{
	m_zoom=zoom;
}
