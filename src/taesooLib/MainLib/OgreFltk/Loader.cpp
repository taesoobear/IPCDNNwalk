#include "stdafx.h"
#ifndef NO_GUI
#include "Loader.h"
#include "../BaseLib/motion/FootPrint.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "MotionManager.h"
#include "../BaseLib/motion/ConstraintMarking.h"
#include "../BaseLib/motion/IKSolver.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/math/intervals.h"
#include "FlLayout.h"
#include <Fl/Fl_File_Chooser.H>

#include "../WrapperLua/luna.h"
#include "../WrapperLua/luna_baselib.h"
#include "../WrapperLua/luna_mainlib.h"
#include "../WrapperLua/LUAwrapper.h"

#define getGroup() ((Loader_wrap*)mGroup)

extern int mScaleFactor;
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
static void computeTargetPos(Motion& mot, Motion& mot2);
class Loader_wrap: public Fl_Group, public FlCallee
{
	Loader* _L;
	public:
#ifndef NO_GUI
	Fl_Output* mText ;
#endif
	FlMenu m_menuExperiment;

	int _work(TString const& workname, TString const& arg);
	Loader_wrap(int x,int y, int w, int h, Loader* l):_L(l),Fl_Group(x,y,w,h){
		_L->mGroup=this;
		resizable(this);
		_L->L=new LUAwrapper();
		LUAwrapper*L=_L->L;
		Register_baselib(L->L);
		Register_mainlib(L->L);
		lunaStack ls(L->L);
		ls.set<Loader>("loader", _L);
		Fl_Button* o;
		if(x==0)
			o=new Fl_Button(x+120*mScaleFactor,y,w-120*mScaleFactor,20*mScaleFactor,"Load");
		else
			o=new Fl_Button(x+120*mScaleFactor,y,60*mScaleFactor,20*mScaleFactor,"Load");
		//o->tooltip("alt+L");
		o->labelsize(11);
		connect(o, Hash("LOAD"));
		mText = new Fl_Output(x+0,y+20*mScaleFactor,80*mScaleFactor,20*mScaleFactor);
		mText->align(FL_ALIGN_BOTTOM);
		mText->textsize(11);
		mText->hide();
		L->dofile((RE::renderer().taesooLibPath()+"Resource/loader.lua").c_str());
		connect(m_menuExperiment);
		end();
	}
	virtual ~Loader_wrap() {delete _L;}
	void onCallback(Fl_Widget* pWidget, int userData)
	{
		_L->onCallback(pWidget, userData);
	}

	bool isMotionDOF(int eExperiment)
	{
		TString test=m_menuExperiment.menu().text(eExperiment);
		lunaStack l(_L->L->L);
		l.getglobal("isMotionDOF");
		l<<test;
		l.call(1,1);
		bool isMotionDOF;
		l>> isMotionDOF;
		return isMotionDOF;
	}
	int work(TString const& workname, lunaStack& L);
};
Loader::Loader(int x, int y, int w, int h, EventReceiver* pEventReceiver):
m_pEventReceiver(pEventReceiver)
{
	//mbAutoLoad=false;
	//BaselibLUA::addWorker(this);


	Loader_wrap* grp=new Loader_wrap(x,y,w,h, this);

}

void Loader::changeFactory(MotionLoader* skeleton, const char* type_name)
{
	TString tn=type_name;
}

Loader::~Loader(void)
{
#ifdef USE_LUABIND
	delete L;
#endif
}

int Loader::numCharacter(int eExperiment)
{
	TString test=getGroup()->m_menuExperiment.menu().text(eExperiment);
#ifdef USE_LUABIND
	int numCtr=luabind::call_function<int>(L->L, "numCharacter", test.ptr());
#else
	lunaStack l(L->L);
	l.getglobal("numCharacter");
	l<<test;
	l.call(1,1);
	int numCtr;
	l>> numCtr;
#endif
	return numCtr;
}


void Loader::startManualLoad(int numCharacters, std::vector<Motion*>& motions)
{
	getGroup()->mText->value("Loading");
	getGroup()->mText->redraw();

	if(m_pEventReceiver)
		m_pEventReceiver->OnLoadStart(numCharacters, motions);


}

void Loader::endManualLoad(std::vector<Motion*> const& motions)
{
	if(motions.size()==1)
		getGroup()->mText->value(TString(sz0::format("%s loaded", motions[0]->GetIdentifier())));
	else
		getGroup()->mText->value(TString(sz0::format("%s,%s loaded", motions[0]->GetIdentifier(), motions[1]->GetIdentifier())));

	if(m_pEventReceiver)
		m_pEventReceiver->OnLoadEnd();
}


void Loader::onCallback(Fl_Widget* pWidget, int userData)
{
	//if(m_menuExperiment==pWidget)
	if(userData==Hash("LOAD"))
	{
		int eExperiment=getGroup()->m_menuExperiment.menu().value();

		if(!getGroup()->m_menuExperiment.isValidItem(eExperiment))
		{
			Msg::msgBox("Motion is not selected");
			return;
		}

		if(eExperiment==getGroup()->m_menuExperiment.size()-1)
		{
			// open from file
			/* // use multi_chooser
			Fl_File_Chooser fc("../Resource/motion", "Motion Files (*.{wrl,asf,vsk,bvh,mot})", Fl_File_Chooser::MULTI, "Choose a file to open");
			fc.show();
			while (fc.visible())
				Fl::wait();

			if (fc.count() > 0)
			*/
			// use native file chooser
			TString new_script=FlChooseFile("Motion Files (*.{wrl,asf,vsk,bvh,mot})", "../Resource/motion", "*.{wrl,asf,vsk,bvh,mot})");
			if(new_script.length())
			{
				getGroup()->mText->value("Loading");
				getGroup()->mText->redraw();
				//for(int i=1; i<=fc.count(); i++)
				{
					//TString filename=fc.value(i);
					TString filename=new_script;
					MotionLoader * pLoader=MotionManager::createMotionLoader(filename);
					TString ext=filename;
					ext=ext.right(3);
					if (ext.toUpper()=="WRL")
					{
						// load dof files
						// open from file
						TString motPath=sz1::parentDirectory(filename);
						TString motFile=FlChooseFile("Motion Files (*.dof)", motPath, "*.dof");
						if(motFile.length())
						{
							int numCtr=1;
							std::vector<MotionDOFinfo const*> aInfo;
							std::vector<MotionDOFcontainer*> motions;
							TString fileName=motFile;//fc.value(0);
							TString mot_id=fileName;

							MotionLoader* skel=pLoader;
							aInfo.resize(1);
							aInfo[0]=&skel->dofInfo;

							if(m_pEventReceiver)
								m_pEventReceiver->OnLoadStart(numCtr, aInfo, motions);

							motions[0]->loadMotion(fileName);

							TString out;
							out.format("%s loaded", mot_id.ptr());
							getGroup()->mText->value(out);

							fileName.op0(sz0::filename());
							if(m_pEventReceiver)
								m_pEventReceiver->OnLoadEnd();
						}
					}
					else
					{
						std::vector<Motion*> motions;
						if(m_pEventReceiver)
							m_pEventReceiver->OnLoadStart(1, motions);

						mTargetMotion=motions[0];
						mTargetMotion->Init(pLoader);


						TString fileName=filename;
						fileName.op0(sz0::filename());
						mTargetMotion->SetIdentifier(fileName.left(-4));

						if(mTargetMotion->numFrames()==0){
							TString ext=filename;
							ext=ext.right(3);

							if(ext.toUpper()=="VSK"){
								// open from file
								Fl_File_Chooser fc("./", "Motion Files (*.v)", Fl_File_Chooser::MULTI, "Choose a file to open");
								fc.show();
								while (fc.visible())
									Fl::wait();

								if (fc.count() > 0) {
									pLoader->loadAnimation(*mTargetMotion, fc.value(0));


									TString fileName=fc.value(0);
									fileName.op0(sz0::filename());
									mTargetMotion->SetIdentifier(fileName.left(-4));
								}
							}
							else if(ext.toUpper()=="ASF"){
								TString file_name=filename;
								file_name=file_name.left(file_name.length()-3);
								file_name+="AMC";
								if(!IsFileExist(file_name))
								{
									TString motPath=sz1::parentDirectory(filename);
									TString motFile=FlChooseFile("Motion Files (*.amc)", motPath, "*.amc");
									if(motFile.length())
									{
										pLoader->loadAnimation(*mTargetMotion, motFile);
										mTargetMotion->SetIdentifier(motFile.left(-4));
									}
								}
							}
						}

						getGroup()->mText->value(TString(sz0::format("%s loaded", mTargetMotion->GetIdentifier())));

						if(m_pEventReceiver)
							m_pEventReceiver->OnLoadEnd();
					}

				}
			}

			return;
		}

		int numCtr=numCharacter(eExperiment);
		bool isMotionDOF=getGroup()->isMotionDOF(eExperiment);
		if (isMotionDOF)
		{
			assert(numCtr==1);
			std::vector<MotionDOFinfo const*> aInfo;
			std::vector<MotionDOFcontainer*> motions;
			TString mot_id=getGroup()->m_menuExperiment.menu().text(eExperiment) ;

			lunaStack l(L->L);
			l.getglobal("isMotionDOF");
			l<<mot_id;
			int n=l.beginCall(1);
			bool isMotionDOF;
			TString fn;
			l>> isMotionDOF>>fn;
			l.endCall(n);

			MotionLoader* skel=RE::motionLoader(fn);
			aInfo.resize(1);
			aInfo[0]=&skel->dofInfo;

			if(m_pEventReceiver)
				m_pEventReceiver->OnLoadStart(numCtr, aInfo, motions);

			l.getglobal("load1");
			l<<mot_id;
			l.push<MotionDOFcontainer>(motions[0]);
			l.call(2,0);
			TString out;
			out.format("%s loaded", mot_id.ptr());
			getGroup()->mText->value(out);
		}
		else
		{
			std::vector<Motion*> motions;
			if(m_pEventReceiver)
				m_pEventReceiver->OnLoadStart(numCtr, motions);

			if(numCtr==2)
			{
				mTargetMotion=motions[0];
				mTargetMotion2=motions[1];

#ifdef USE_LUABIND
				luabind::call_function<void>(L->L, "load2", m_menuExperiment.menu().text(eExperiment),
						boost::ref(*mTargetMotion), boost::ref(*mTargetMotion2));
#else
				lunaStack l(L->L);
				l.getglobal("load2");
				TString mot_id=getGroup()->m_menuExperiment.menu().text(eExperiment) ;
				l<<mot_id;
				l.push<Motion>(mTargetMotion);
				l.push<Motion>(mTargetMotion2);
				l.call(3,0);
#endif
				/*			computeTargetPos(*mTargetMotion, *mTargetMotion2);
							computeTargetPos(*mTargetMotion2, *mTargetMotion);
							mTargetMotion->CalcInterFrameDifference();
							mTargetMotion2->CalcInterFrameDifference();*/
				getGroup()->mText->value(TString(sz0::format("%s,%s loaded", mTargetMotion->GetIdentifier(), mTargetMotion2->GetIdentifier())));
			}
			else
			{
				mTargetMotion=motions[0];
				mTargetMotion2=NULL;
#ifdef USE_LUABIND
				luabind::call_function<void>(L->L, "load1", m_menuExperiment.menu().text(eExperiment), boost::ref(*mTargetMotion));
#else
				lunaStack l(L->L);
				l.getglobal("load1");
				TString mot_id=getGroup()->m_menuExperiment.menu().text(eExperiment) ;
				l<<mot_id;
				l.push<Motion>(mTargetMotion);
				l.call(2,0);
#endif
				getGroup()->mText->value(TString(sz0::format("%s loaded", mTargetMotion->GetIdentifier())));
			}

		}
		if(m_pEventReceiver)
			m_pEventReceiver->OnLoadEnd();
	}
}

void Loader::calcInterCon()
{
	mTargetMotion->CalcInterFrameDifference();
	constraintAutomaticMarking(*mTargetMotion);
	if(mTargetMotion2 && mTargetMotion2->numFrames())
	{
		mTargetMotion2->CalcInterFrameDifference();
		constraintAutomaticMarking(*mTargetMotion2);
	}

}

void computeTargetPos(Motion& mot, Motion& pairMot)
{
	assert(mot.numFrames()==pairMot.numFrames());

	/*
	int i;
	for(i=0;i<mot.numFrames();++i){
		vector3 opponentPos;
		mot.pose(i).decomposeRot();
		opponentPos.difference(mot.pose(i).m_aTranslations[0], pairMot.pose(i).m_aTranslations[0]);
		opponentPos.y=0;
		mot.pose2(i).m_targetDir=opponentPos;
		opponentPos.rotate(mot.pose2(i).m_realRotAxis_y.inverse());
		mot.pose2(i).m_oppenentPos=opponentPos;
	}
	*/
}

int addItemRecursive(int curItem, bool bSubMenu, FlMenu & menu, lunaStack& ll, int tblindex);
int Loader::work(TString const& workname, lunaStack& L)
{
	if (workname=="load")
		onCallback(NULL,Hash("LOAD"));
	else
		return getGroup()->work(workname, L);
}
int Loader::_work(TString const& workname, TString const& arg)
{
	return getGroup()->_work(workname, arg);
}
int Loader_wrap::_work(TString const& workname, TString const& arg)
{
	if (workname=="setDefault")
	{
		for(int i=0; i<m_menuExperiment.menu().size(); i++)
		{
			if(arg==m_menuExperiment.menu().text(i))
			{
				m_menuExperiment.menu().value(i);
			}
		}
	}
	return 1;
}
int Loader_wrap::work(TString const& workName, lunaStack& L)
{

/*	if(workName=="setAutoLoad")
	{
		mbAutoLoad=luabind::object_cast<bool>(table);
	}
	else */
	if(workName=="setDefault")
	{
		TString itemt;
		L>>itemt;
		_work(workName, itemt);
	}
	else if(workName=="addItem")
	{
		m_menuExperiment.initChoice(x(),y(),120*mScaleFactor,20*mScaleFactor,"Experiment");

		//L.printStack();	
		int tblindex=L.gettop();
		int numMenuItems=L.treeSize(tblindex)-1;
		m_menuExperiment.size(numMenuItems+1);

		addItemRecursive(0, false, m_menuExperiment, L, tblindex);
		/*
		int i=0;
		for(i=0; i<numMotions; i++)
		{
			m_menuExperiment.item(i, luabind::object_cast<const char*>(table[i+1])); // lua indexing starts from 1.
		}*/
		m_menuExperiment.item(numMenuItems, "Load from file");
	}
/*
	else if(workName=="getTargetMotion")
	{
		int target;
		lua>>target;
		BaselibLUA::LMotion* mot=lua.pop<BaselibLUA::LMotion>();
		if(target==1)
			mot->mMotion=mTargetMotion;
		else
			mot->mMotion=mTargetMotion2;

		mot->mbReference=true;
	}*/
	return 1;
}

void calcConstraintPos(Motion const& mot, int constraint, intIntervals& conFrames, matrixn& conPositions, vectorn& conLen)
{
	MotionUtil::GetSignal getSignal(mot);

	bitvectorn conToe;
	getSignal.constraint(constraint, conToe);

	intvectorn conInterval;

	MotionUtil::SegmentFinder sf(mot, 0, mot.numFrames());

	conFrames.setSize(0);
	for(int iseg=0; iseg<sf.numSegment(); iseg++)
	{
		int startSeg=sf.startFrame(iseg);
		int endSeg=sf.endFrame(iseg);

		conInterval.runLengthEncode(conToe, startSeg, endSeg);
		int numConGrp=conInterval.size()/2;

		for(int grp=0; grp<numConGrp; grp++)
		{
			int start=conInterval[grp*2];
			int end=conInterval[grp*2+1];

			conFrames.pushBack(start, end);
		}
	}

	conPositions.setSize(conFrames.size(), 3);
	conLen.setSize(conFrames.size());

	vectorn speed;
	matrixn conPos;
	vectorn length;
	vector3 pos;
	for(int coni=0; coni<conFrames.size(); coni++)
	{
		int cs=conFrames.start(coni);

		conPos.setSize(conFrames.end(coni)-cs,3);
		length.setSize(conFrames.end(coni)-cs);

		for(int i=cs; i<conFrames.end(coni); i++)
		{
			mot.skeleton().setPose(mot.pose(i));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(pos);
			conPos.row(i-cs).assign(pos);
			length[i-cs]=MotionUtil::calcIKlength(mot.skeleton(), constraint);
		}

		speed.setSize(conFrames.end(coni)-cs-1);
		for(int i=cs; i<conFrames.end(coni)-1; i++)
		{
			vector3 vel;
			vel.difference(conPos.row3(i-cs),
							conPos.row3(i-cs+1));

			speed[i-cs]=vel.length();
		}

		int argmin=speed.argMin();
		conPositions.row(coni).assign(conPos.row(argmin));
		if(length.size()==1)
			conLen[coni]=length[argmin];
		else
			conLen[coni]=MAX(length[argmin], length[argmin+1]);
	}


}


void storeConstraintPos(Motion & mot, int constraint, intIntervals& conFrames, matrixn& conPositions)
{
	int numConGrp=conFrames.size();
	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=conFrames.start(grp);
		int end=conFrames.end(grp);
		vector3 conPos=conPositions.row3(grp);

		for(int i=start; i<end; i++)
		{
			MotionUtil::ConstraintMarking::encodeCon(constraint, mot.pose(i), mot.skeleton(), conPos);
		}
	}

	MotionUtil::SegmentFinder sf(mot, 0, mot.numFrames());

	for(int iseg=0; iseg<sf.numSegment(); iseg++)
	{
		int startSeg=sf.startFrame(iseg);
		int endSeg=sf.endFrame(iseg);


		// fill gap (linearly blend constraint positions inbetween constrained frames.)

		bool bCont=true;
		for(int grp=0; grp<=numConGrp; grp++)
		{
			if(grp!=numConGrp && conFrames.end(grp)<startSeg)
			{
				bCont=true;
				continue;
			}


			int prevEnd, nextStart;
			vector3 prevConPos, nextConPos;
#define CONTOE(y, x) if(constraint==CONSTRAINT_LEFT_TOE) y=mot.pose(x).m_conToeL;\
				else y=mot.pose(x).m_conToeR

			if(bCont)
			{
				prevEnd=startSeg;
				bCont=false;
				CONTOE(prevConPos, prevEnd);
			}
			else
			{
				prevEnd=conFrames.end(grp-1);
				CONTOE(prevConPos, prevEnd-1);
			}

			if(grp==numConGrp|| conFrames.start(grp)>=endSeg)
			{
				nextStart=endSeg;
				CONTOE(nextConPos, nextStart-1);
			}
			else
			{
				nextStart=conFrames.start(grp);
				CONTOE(nextConPos, nextStart);
			}

			for(int i=prevEnd; i<nextStart; i++)
			{
				vector3 toepos;
				toepos.interpolate( (m_real)(i-prevEnd+1)/(m_real)(nextStart-prevEnd+1), prevConPos, nextConPos);

				if(constraint==CONSTRAINT_LEFT_TOE)
					mot.pose(i).m_conToeL=toepos;
				else
					mot.pose(i).m_conToeR=toepos;
			}
			if(grp==numConGrp|| conFrames.start(grp)>=endSeg) break;
		}
	}
}

void Loader::constraintAutomaticMarking(Motion& mot)
{
	LUAwrapper L;
#ifdef USE_LUABIND
	L.setVal<const char*>("motionId", mot.GetIdentifier());
#else
	lunaStack l(L.L);
	l<<"motionId"<<mot.GetIdentifier() ;
	l.settable();
#endif
	L.dofile("../resource/constraint.lua");

	double toe_height_thr;
	double toe_speed_thr;
	double heel_height_thr;
	double heel_speed_thr;
	int eHowToChooseConstraint;
	int bCleanup;
	int bFillGap;
	int mincon;
	int reducecon;
	bool bIKtest;

#ifdef USE_LUABIND
	L.getVal<double>( "toe_height_thr", toe_height_thr);
	L.getVal<double>( "toe_speed_thr", toe_speed_thr);
	L.getVal<double>( "heel_height_thr", heel_height_thr);
	L.getVal<double>( "heel_speed_thr",heel_speed_thr);
	L.getVal<int>( "howToChooseConstraint", eHowToChooseConstraint);
	L.getVal<int>( "cleanup", bCleanup);
	L.getVal<int>(  "fillGap", bFillGap);

	L.getVal<int>(  "minConDuration", mincon);
	L.getVal<int>(  "reduceCon", reducecon);
	L.getVal<bool>("checkIKPossible", bIKtest);
#else

	assert(false);
#endif


	MotionUtil::ConstraintMarking cm(&mot, bCleanup, bFillGap, mincon, reducecon);
	cm.calcConstraint(toe_height_thr, toe_speed_thr, heel_height_thr, heel_speed_thr, true, eHowToChooseConstraint);


	if(bIKtest)
	{

		intIntervals lconi, rconi;
		matrixn conPositionsL, conPositionsR;
		vectorn conLenL, conLenR;

		double lengthGoal, distGoal;
#ifdef USE_LUABIND
		L.getVal<double>("checkIKPossible_lengthGoal",  lengthGoal);
		L.getVal<double>("checkIKPossible_distGoal",  distGoal);
#else
		assert(false);
#endif

		calcConstraintPos(mot, CONSTRAINT_LEFT_TOE, lconi, conPositionsL, conLenL);
		calcConstraintPos(mot, CONSTRAINT_RIGHT_TOE, rconi, conPositionsR, conLenR);

		for(int icon=0; icon<lconi.size(); icon++)
		{
			for(int i=lconi.start(icon); i<lconi.end(icon); i++)
			{
				vector3 conpos;

				mot.setSkeleton(i);

				if(mot.isConstraint(i, CONSTRAINT_LEFT_TOE))
				{
					conpos=conPositionsL.row3(icon);
					if(!MotionUtil::isIKpossible(mot.skeleton(), CONSTRAINT_LEFT_TOE, conpos, MAX(lengthGoal, conLenL[icon]), distGoal))
						mot.setConstraint(i, CONSTRAINT_LEFT_TOE, false);
				}
			}
		}

		for(int icon=0; icon<rconi.size(); icon++)
		{
			for(int i=rconi.start(icon); i<rconi.end(icon); i++)
			{
				vector3 conpos;

				mot.setSkeleton(i);

				if(mot.isConstraint(i, CONSTRAINT_RIGHT_TOE))
				{
					conpos=conPositionsR.row3(icon);
					if(!MotionUtil::isIKpossible(mot.skeleton(), CONSTRAINT_RIGHT_TOE, conpos, MAX(lengthGoal, conLenR[icon]), distGoal))
						mot.setConstraint(i, CONSTRAINT_RIGHT_TOE, false);
				}
			}
		}

		bitvectorn lcon, rcon;
		MotionUtil::GetSignal gs(mot);
		gs.constraint(CONSTRAINT_LEFT_TOE, lcon);
		gs.constraint(CONSTRAINT_RIGHT_TOE, rcon);
		// 완전히 날라간 constraint살려주기.
		for(int i=0; i<lconi.size(); i++)
		{
			if(lcon.find(lconi.start(i), lconi.end(i))==lconi.end(i))
			{
				int cont=(lconi.start(i)+lconi.end(i))/2;
				printf("recover con %d\n", cont);
				mot.setConstraint(cont, CONSTRAINT_LEFT_TOE);
			}
		}

		for(int i=0; i<rconi.size(); i++)
		{
			if(rcon.find(rconi.start(i), rconi.end(i))==rconi.end(i))
			{
				int cont=(rconi.start(i)+rconi.end(i))/2;
				printf("recover con %d\n", cont);
				mot.setConstraint(cont, CONSTRAINT_RIGHT_TOE);
			}
		}

		// constraint정보 저장하기.
		calcConstraintPos(mot, CONSTRAINT_LEFT_TOE, lconi, conPositionsL, conLenL);
		calcConstraintPos(mot, CONSTRAINT_RIGHT_TOE, rconi, conPositionsR, conLenR);

		storeConstraintPos(mot, CONSTRAINT_LEFT_TOE, lconi, conPositionsL);
		storeConstraintPos(mot, CONSTRAINT_RIGHT_TOE, rconi, conPositionsR);
	}
}

#endif
