#include <vector>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/lu.hpp"


/*
 * Function for calculating determinant of a nxn matrix, taken from:
 * http://stackoverflow.com/questions/29877760/boostublas-how-to-get-determinant-of-int-matrix
 */
/*template<typename ValType>
ValType det_fast(const boost::numeric::ublas::matrix<ValType>& matrix)
{
    // create a working copy of the input 
    boost::numeric::ublas::matrix<ValType> mLu(matrix);
    boost::numeric::ublas::permutation_matrix<std::size_t> pivots(matrix.size1());

    unsigned int isSingular = boost::numeric::ublas::lu_factorize(mLu, pivots);
    if (isSingular)
        return static_cast<ValType>(0);

    ValType det = static_cast<ValType>(1);
    for (std::size_t i = 0; i < pivots.size(); ++i) 
    {
        if (pivots(i) != i)
            det *= static_cast<ValType>(-1);

        det *= mLu(i, i);
    }

    return det;
}
*/
/*
 *	Uses repeated squaring to quickly raise a number to a power
 *  http://stackoverflow.com/questions/101439/the-most-efficient-way-to-implement-an-integer-based-power-function-powint-int
 */
double fastPow(double base, unsigned int exponent)
{
	double result = 1.0;
	while (exponent)
	{
		if (exponent & 1)
		{
			result = result * base;
		}
		exponent = exponent >> 1;
		base = base * base;
	}
	return result;
}

class RandomVectorAnalysis
{
private:
	unsigned int numDimensions;
	std::vector<std::string> meanings;

	class Sample
	{
	public:
		boost::posix_time::ptime timestamp; 
		std::vector<double> values;
	};

	std::vector<Sample> history;
	std::vector<double> mean;
	boost::numeric::ublas::matrix<double> covarianceMatrix;
	boost::numeric::ublas::permutation_matrix<std::size_t> pm;
	bool isSingular;
	double determinant;
	bool cached;

public:
	RandomVectorAnalysis(std::vector<std::string> dimensionMeanings)
		: covarianceMatrix(dimensionMeanings.size(), dimensionMeanings.size()), pm(covarianceMatrix.size1())
	{
		numDimensions = dimensionMeanings.size();
		meanings = dimensionMeanings;
		mean.resize(numDimensions);
		isSingular = true;
		determinant = 1.0;
		cached = false;
	}

	bool addToHistory(std::vector<double> sample)
	{
		if (sample.size() != numDimensions)
		{
			printf("numDimensions = %u\n", numDimensions);
			return false;
		}
		cached = false;
		Sample i;
		i.values = sample;
		i.timestamp = boost::posix_time::microsec_clock::local_time();
		history.push_back(i);
		return true;
	}

	double getCovarianceDistance(std::vector<double> sample)
	{
		if (sample.size() != numDimensions)
		{
			printf("Sample has %lu dimensions, %u expected.\n", sample.size(), numDimensions);
			return 0.0;
		}

		//printf("History size: %lu\n", history.size());
		if (history.size() <= 1)
		{
			return 0.0;
		}

		//Have we already done the calculations?
		if (!cached)
		{
			//Calculate the mean
			
			//printf("History:\n");
			//For each sample in the history
			for (unsigned int  s = 0; s < history.size(); ++s)
			{
				//For each dimension
				for (unsigned int  d = 0; d < numDimensions; ++d)
				{
					mean[d] = mean[d] + history[s].values[d];
					//printf("%f, ", history[s].values[d]);
				}
				//printf("\n");
			}
			//For each dimension
			for (unsigned int  d = 0; d < numDimensions; ++d)
			{
				mean[d] = mean[d] / history.size();
			}


			//Calculate the deviation history matrix
			std::vector< std::vector<double> > deviationHistory(history.size());
			//For each row
			for (unsigned int  r = 0; r < history.size(); ++r )
			{
				std::vector<double> row(numDimensions);
				for (unsigned int  d = 0; d < numDimensions; ++d )
				{
					row[d] = history[r].values[d] - mean[d];
				}
				deviationHistory[r] = row;
			}

			//printf("Covariance Matrix:\n");
			//Calculate the covariance matrix
			//For each row
			for (unsigned int  r = 0; r < numDimensions; ++r )
			{
				//For each column
				for (unsigned int  c = 0; c < numDimensions; ++c)
				{
					covarianceMatrix(r, c) = 0;
					//For each element in the deviation history
					for (unsigned int  j = 0; j < deviationHistory.size(); ++j )
					{
						covarianceMatrix(r,c) = covarianceMatrix(r,c) + deviationHistory[j][r] * deviationHistory[j][c];
					}
					covarianceMatrix(r,c) = covarianceMatrix(r,c) / deviationHistory.size();
					//printf("%f, ", covarianceMatrix(r,c));
				}
				//printf("\n");
			}

			//Factorize the covariance matrix. This function modifies the covariance matrix
			pm = boost::numeric::ublas::permutation_matrix<std::size_t>(covarianceMatrix.size1());
			isSingular = boost::numeric::ublas::lu_factorize(covarianceMatrix, pm);
			if (!isSingular)
			{
				//Calculate the determinant
				//http://stackoverflow.com/questions/29877760/boostublas-how-to-get-determinant-of-int-matrix
				determinant = 1.0;
				for (std::size_t i = 0; i < pm.size(); ++i)
			    {
			        if (pm(i) != i)
			            determinant *= -1.0;

			        determinant *= covarianceMatrix(i, i);
			    }
			    //printf("Determinant: %f\n", determinant);
			} else {
				//printf("Singular Matrix!\n");
				determinant = 0;
			}
			
			cached = true;
		}

		double distance = 0.0;
		//Check that there is some variance in all of the dimensions
		/*for (unsigned int d = 0; d < numDimensions; ++d)
		{
			if (covarianceMatrix(d,d) == 0.0)
			{
				return 0.0;
			}
		}*/
		//Check for singularity
		if (!isSingular)
		{
			//Standardize the sample by subtracting the mean from the sample
			boost::numeric::ublas::matrix<double> standardizedSample(numDimensions, 1);
			for (unsigned int d = 0; d < numDimensions; ++d)
			{
				standardizedSample(d, 0) = sample[d] - mean[d];
			}

			//Now calculate the multivariate distance measure
			//distance = transpose(standardizedSample) * inverse(covarianceMatrix) * standardizedSample
			
			//First calculate inverse(covarianceMatrix) * standardized sample,
			//which is equivalent to solving covarianceMatrix * x = standardizedSample
			boost::numeric::ublas::matrix<double> partialSolution = standardizedSample;
			boost::numeric::ublas::lu_substitute(covarianceMatrix, pm, partialSolution);
			
			//Now solve transpose(standardizedSample) * partialSolution
			for (unsigned int d = 0; d < numDimensions; ++d)
			{
				distance = distance + partialSolution(d, 0) * standardizedSample(d, 0);
			}

			/*double exponent = -0.5 * distance;
			printf("Exponent: %f\n", exponent);

			printf("2pi^%d: %f\n", numDimensions, fastPow(2 * 3.14159265359, numDimensions));

			//Plug into the equation
			double pdf = exp(exponent) / (sqrt(determinant * fastPow(2 * 3.14159265359, numDimensions)));
			*/
		}
		return distance;		
	}

	double getKernelProbabilityOfSeeing(std::vector<double> sample)
	{
		if (sample.size() != numDimensions)
		{
			return 0.0;
		}
		return 0.0;
	}
};