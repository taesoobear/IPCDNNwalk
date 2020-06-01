#pragma once


class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
namespace MotionUtil
{
	class FootPrint
	{
	public:
		FootPrint(){}
		virtual ~FootPrint(){}
		virtual void getFootPrints(const Motion& mot, int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const=0;
		void getFootInterval(const Motion& mot, int iframe, int constraint, int& left, int& right) const;
	protected:
	};

	/// CalcFootPrint calculated the constrain position as the average of the foot positions.
	class CalcFootPrint : public FootPrint
	{
	public:
		CalcFootPrint(){}
		virtual ~CalcFootPrint(){}
		virtual void getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const;
	};

	/// GetFootPrint uses the pre-stored foot position information in the Poses.
	class GetFootPrint : public FootPrint
	{
	public:
		GetFootPrint(){}
		virtual ~GetFootPrint(){}
		virtual void getFootPrints(const Motion& mot, int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const;		
	};


	/// GetFootPrint uses the pre-stored foot position information in the Poses.
	class GetFootPrintOnline : public FootPrint
	{
		interval mHeightInterval;
		m_real mLengthThr;
		m_real mDistThr;
	public:
		GetFootPrintOnline(m_real minHeight=0.0, m_real maxHeight=4.0, m_real lengthThr=0.95, m_real distThr=0.3);
		virtual ~GetFootPrintOnline(){}
		virtual void getFootPrints(const Motion& mot, int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const;		
	};

	/// CalcFootPrint calculated the constrain position as the position of minimum velocity moment.
	class CalcFootPrintSpeed : public FootPrint
	{
	public:
		CalcFootPrintSpeed (){}
		virtual ~CalcFootPrintSpeed (){}
		virtual void getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const;
	};


	class CalcFootPrintOnline : public FootPrint
	{
		bool m_bSupportFootOnly;
	public:
		CalcFootPrintOnline (bool bSupportFootOnly=false):m_bSupportFootOnly(bSupportFootOnly){}
		virtual~CalcFootPrintOnline (){}

		// start에 가까운 데서 시작하는 foot print는 앞쪽위치로 준다. 딴놈들은 센터에서 구하고, 마지막 놈은 마지막 프레임으로 한다.
		virtual void getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& interval, matrixn& aFootPositions) const;

		static void heuristicFootDetection(const Motion& mot, int start, int end, int constraint);
	};

}
