
#ifndef CLAPACK_WRAP
#define CLAPACK_WRAP

namespace CLapack
{
void balancedSchurDecomposition(matrixn const& H, matrixn& u);
 void LUinvert(matrixn& out, const matrixn& in);

}
#endif
