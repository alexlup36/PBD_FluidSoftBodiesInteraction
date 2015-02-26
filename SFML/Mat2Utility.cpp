#include "Mat2Utility.h"

void JacobiRotate(glm::mat2& A, glm::mat2& R)
{
	float d = (A[0][0] - A[1][1]) / (2.0f * A[0][1]);
	float t = 1.0f / (fabs(d) + sqrt(d * d + 1.0f));
	if (d < 0.0f)
	{
		t = -t;
	}
	float c = 1.0f / sqrt(t * t + 1.0f);
	float s = t * c;
	A[0][0] += (t * A[0][1]);
	A[1][1] -= (t * A[0][1]);
	A[0][1] = 0.0f;
	A[1][0] = 0.0f;

	for (int k = 0; k < 2; k++)
	{
		float Rkp = c * R[k][0] + s * R[k][1];
		float Rkq = -s * R[k][0] + c * R[k][1];
		R[k][0] = Rkp;
		R[k][1] = Rkq;
	}
}

void EigenDecomposition(glm::mat2& A, glm::mat2& R)
{
	R = glm::mat2(1.0f);
	JacobiRotate(A, R);
}

void PolarDecomposition(const glm::mat2& A, glm::mat2& R, glm::mat2& S)
{
	// A = RS S symmetric, R orthonormal
	// S = (ATA)^(1/2)

	R = glm::mat2(1.0f);

	glm::mat2 ata = glm::transpose(A) * A;

	glm::mat2 U;
	EigenDecomposition(ata, U);

	float l0 = ata[0][0];
	if (l0 <= 0.0f)
	{
		l0 = 0.0f;
	}
	else
	{
		l0 = 1.0f / sqrt(l0);
	}

	float l1 = ata[1][1];
	if (l1 <= 0.0f)
	{
		l1 = 0.0f;
	}
	else
	{
		l1 = 1.0f / sqrt(l1);
	}

	glm::mat2 s1;
	s1[0][0] = l0 * U[0][0] * U[0][0] + l1 * U[0][1] * U[0][1];
	s1[0][1] = l0 * U[0][0] * U[1][0] + l1 * U[0][1] * U[1][1];
	s1[1][0] = s1[0][1];
	s1[1][1] = l0 * U[1][0] * U[1][0] + l1 * U[1][1] * U[1][1];

	R = A * s1;
	S = glm::transpose(R) * A;
}
