#ifndef TRAITS_H_
#define TRAITS_H_
#pragma once

class vector2;
class vector3;
class quater;
namespace baselib
{
	template <typename T, typename T_base> 
	class traits
	{
	public:
		inline static T zero() { return T();}
		inline static T_base distance(T a, T b)
		{
			return T_base(ABS(a-b));
		}
	};
	template <> 
	class traits<m_real, m_real>
	{
	public:
		inline static m_real zero() { return 0.0;}
		inline static m_real distance(m_real a, m_real b) { return m_real(ABS(a-b));}
	};
	template <> 
	class traits<int, int>
	{
	public:
		inline static int zero() { return 0;}
	};
	template <> 
	class traits<vector3, m_real>
	{
	public:
		inline static vector3 zero() {return vector3(0.0,0.0,0.0);}
	};
	
	template <> 
	class traits<quater, m_real>
	{
	public:
		inline static quater zero() {return quater(1.0,0.0,0.0,0.0);}
	};
}
#endif
