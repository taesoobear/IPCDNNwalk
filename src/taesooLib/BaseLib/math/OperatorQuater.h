#pragma once

namespace v2
{
	struct interpolateQuater: public _op
	{
		interpolateQuater (m_real t):m_fT(t){}
		virtual void calc(vectorn& c, const vectorn& a, const vectorn& b) const;
		m_real m_fT;
	};
}
