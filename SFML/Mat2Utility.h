#ifndef MAT2UTILITY_H
#define MAT2UTILITY_H

#include "Common.h"

void JacobiRotate(glm::mat2& A, glm::mat2& R);
void EigenDecomposition(glm::mat2& A, glm::mat2& R);
void PolarDecomposition(const glm::mat2& A, glm::mat2& R, glm::mat2& S);

#endif // MAT2UTILITY_H