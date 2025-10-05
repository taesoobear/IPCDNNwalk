#include "stdafx.h"
#include "./TWord.h"
#include "math/mathclass.h"


TWord::TWord(const char *word)									
{
	m_nWord=0; 
	for(unsigned int i=0; i<MIN(4,strlen(word)); i++) 
		m_aByte[i]=word[i]; 
}


TWord::TWord(unsigned short lowWord, const TString& highWord)	// highword를 char* 로 하면, ambiguity가 생긴다.
{
	m_aWord[0]=lowWord;
	// The only endian dependent code.
	m_aByte[2]=highWord[0];
	m_aByte[3]=highWord[1];
}

/*
int Hash(const char* str)
{
	int n=strlen(str);

	int h = 0;
	for (int i = 0; i < n; i++) 
	{
		h = 31*h + str[i];
	}
	return h;
}
*/

/* We assume to have `unsigned int' value with at least 32 bits.  */
#define HASHWORDBITS 32


/* Defines the so called `hashpjw' function by P.J. Weinberger
[see Aho/Sethi/Ullman, COMPILERS: Principles, Techniques and Tools,
1986, 1987 Bell Telephone Laboratories, Inc.]  */
int Hash(const char *str_param)
{
	unsigned int hval, g;
	const char *str = str_param;

	/* Compute the hash value for the given string.  */
	hval = 0;
	while (*str != '\0')
	{
		hval <<= 4;
		hval += (unsigned int) *str++;
		g = hval & ((unsigned int) 0xf << (HASHWORDBITS - 4));
		if (g != 0)
		{
			hval ^= g >> (HASHWORDBITS - 8);
			hval ^= g;
		}
	}
	return (int)hval;
}