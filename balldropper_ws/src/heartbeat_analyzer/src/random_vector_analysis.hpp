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
	
	//A class to encapsulate a vector of doubles
	class DoubleVector
	{
	public:
		std::vector<double> values;
		double length;
		boost::posix_time::ptime timestamp;

		void computeLength()
		{
			length = 0.0;
			for (int d = 0; d < values.size(); ++d)
			{
				length = values[d] * values[d];
			}
			length = sqrt(length);
		}

		DoubleVector operator - (const DoubleVector& rhs) const
		{
			DoubleVector result;
			result.values.resize(values.size());
			for (int d = 0; d < values.size(); ++d )
			{
				result.values[d] = values[d] - rhs.values[d];
			}
			return result;
		}

		DoubleVector operator + (const DoubleVector& rhs) const
		{
			DoubleVector result;
			result.values.resize(values.size());
			for (int d = 0; d < values.size(); ++d )
			{
				result.values[d] = values[d] + rhs.values[d];
			}
			return result;
		}

		DoubleVector& operator = (const DoubleVector& rhs)
		{
			values = rhs.values;
			return *this;
		}
		
		bool operator < (const DoubleVector& rhs) const
		{
			return length < rhs.length;
		}
	};

	//Compute the nearest neighbor of each point, and return its vector
	static std::vector< DoubleVector > kNearestNeighbors( const std::vector<DoubleVector>& samples, int k  )
	{	
		//Can't have more neighbors than there are points
		if ( k >= samples.size())
		{
			k = samples.size() - 1;
		}

		std::vector<DoubleVector> result(k * samples.size());
		std::vector<DoubleVector> differenceVectors(samples.size());

		//For each sample
		for (int s = 0; s < samples.size(); ++s)
		{
			//Find the distance to each neighbor
			for (int n = 0; n < samples.size(); ++n)
			{
				differenceVectors[n] = samples[n] - samples[s];
				differenceVectors[n].computeLength();
			}
			//Sort the difference vectors in ascending order
			std::sort( differenceVectors.begin(), differenceVectors.end() );

			//Keep the vectors to the k nearest neighbors
			for (int n = 0; n < k; ++n)
			{
				//Skip the 0'th difference vector, because that is the difference to itself.
				result[s * k + n] = differenceVectors[n + 1];
			}
		}
		//Sort the result in ascending order
		std::sort(result.begin(), result.end());
		//Reverse the sort
		std::reverse(result.begin(), result.end());

		return result;
	}


	static void computeMean(const std::vector< DoubleVector >& data, DoubleVector& mean)
	{
		if (data.size() == 0)
		{
			return;
		}
		//Calculate the mean
		mean.values.clear();
		mean.values.resize( data[0].values.size(), 0.0 );
		
		//For each sample in the data
		for (unsigned int  s = 0; s < data.size(); ++s)
		{
			mean = mean + data[s];
		}
		//For each dimension
		for (unsigned int  d = 0; d < mean.values.size(); ++d)
		{
			mean.values[d] = mean.values[d] / data.size();
		}
	}

	static bool computeAndFactorizeCovarianceMatrix(const std::vector< DoubleVector >& data,
													const DoubleVector& mean,
													boost::numeric::ublas::matrix<double>& covarianceMatrix,
													boost::numeric::ublas::permutation_matrix<std::size_t>& pm,
													double& determinant )
	{
		bool isSingular = false;

		if (data.empty())
		{
			return isSingular;
		}

		int numDimensions = data[0].values.size();

		//Calculate the deviation history matrix
		std::vector< DoubleVector > dataDeviation(data.size());
		//For each sample
		for (unsigned int s = 0; s < data.size(); ++s )
		{
			dataDeviation[s] = data[s] - mean;
		}

		//printf("Covariance Matrix:\n");
		//Calculate the covariance matrix
		//For each row
		for (unsigned int  r = 0; r < data[0].values.size(); ++r )
		{
			//For each column
			for (unsigned int  c = 0; c < numDimensions; ++c)
			{
				covarianceMatrix(r, c) = 0;
				//For each element in the deviation history
				for (unsigned int  j = 0; j < dataDeviation.size(); ++j )
				{
					covarianceMatrix(r,c) = covarianceMatrix(r,c) + dataDeviation[j].values[r] * dataDeviation[j].values[c];
				}
				covarianceMatrix(r,c) = covarianceMatrix(r,c) / dataDeviation.size();
				//printf("%f, ", dataCovarianceMatrix(r,c));
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
		return isSingular;
	}

	//Estimate the minimum volume enclosing ellipse
	boost::numeric::ublas::matrix<double> mvee( std::vector< std::vector<double> >& samples )
	{
		//Sort the vectors by their length
		//For every tuple of d vectors, starting with the largest
			//If they are linearly independent
				//Find the transformation matrix to change to their coordinate system.
				//For each other vector
					//Convert it to the coordinate system of the d vectors
					//If its magnitude is less than 1, remove it from the list
		//Use linear regression to find the best-fit matrix that maps the remaining vectors to the unit hypersphere.
	}

	unsigned int numDimensions;
	std::vector<std::string> meanings;
	std::vector<DoubleVector> history;
	DoubleVector mean;
	boost::numeric::ublas::matrix<double> dataCovarianceMatrix;
	boost::numeric::ublas::permutation_matrix<std::size_t> dataPm;
	bool isSingular;
	double determinant;
	bool cached;

public:
	RandomVectorAnalysis(const std::vector<std::string>& dimensionMeanings)
		: dataCovarianceMatrix(dimensionMeanings.size(), dimensionMeanings.size()), dataPm(dataCovarianceMatrix.size1())
	{
		numDimensions = dimensionMeanings.size();
		meanings = dimensionMeanings;
		mean.values.resize(numDimensions);
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
		DoubleVector i;
		i.values = sample;
		i.timestamp = boost::posix_time::microsec_clock::local_time();
		history.push_back(i);
		return true;
	}

	void saveHistory(char* fileName)
	{
		FILE* saveFile = NULL;

		saveFile = fopen( fileName, "w");

		for (int m = 0; m < meanings.size(); ++m )
		{
			fprintf(saveFile, "%s", meanings[m].c_str());
			if (m < meanings.size() - 1)
			{
				fprintf(saveFile, "\t");
			}
			else
			{
				fprintf(saveFile, "\n");
			}
		}

		for (int s = 0; s < history.size(); ++s )
		{
			for (int d = 0; d < history[s].values.size(); ++d )
			{
				fprintf(saveFile, "%f", history[s].values[d]);
				if (d < history[s].values.size() - 1)
				{
					fprintf(saveFile, "\t");
				}
				else
				{
					fprintf(saveFile, "\n");
				}
			}
		}

		fclose(saveFile);
	}

	bool loadHistory(char* fileName)
	{
		FILE* loadFile = NULL;
		loadFile = fopen( fileName, "r");

		if (loadFile != NULL)
		{
			char buffer[512];
			int bufferIter = 0;
			int startOfNextToken = 0;
			int c;

			numDimensions = 0;
			meanings.clear();

			//Read the first line
			while ((c = fgetc(loadFile)) != EOF && bufferIter != sizeof(buffer))
			{
				//If we've read a token
				if (c == '\t' || c == '\n')
				{
					//Save the string
					buffer[bufferIter] = '\0';
					std::string dimensionMeaning(&buffer[startOfNextToken]);
					meanings.push_back(dimensionMeaning);
					++numDimensions;
					startOfNextToken = bufferIter + 1;
				}
				//Copy the read char into the buffer
				buffer[bufferIter] = c;
				++bufferIter;
				if (c == '\n')
				{
					break;
				}
			}

			std::vector<double> sample(numDimensions);
			while (!feof(loadFile) )
			{
				int d;
				for (d = 0; d < numDimensions - 1; ++d)
				{
					fscanf(loadFile, "%lf\t", &sample[d]);
				}
				fscanf(loadFile, "%lf\n", &sample[d]);
				addToHistory( sample );
			}
			fclose(loadFile);
			return true;
		}
		return false;
	}

	double getGaussianDistance(std::vector<double> sample)
	{
		if (sample.size() != numDimensions)
		{
			printf("DoubleVector has %lu dimensions, %u expected.\n", sample.size(), numDimensions);
			return 0.0;
		}

		if (history.size() <= 1)
		{
			return 0.0;
		}

		//Have we already done the calculations?
		if (!cached)
		{
			computeMean(history, mean);
			isSingular = computeAndFactorizeCovarianceMatrix(history, mean, dataCovarianceMatrix, dataPm, determinant);
			cached = true;
		}

		double distance = 0.0;

		if (!isSingular)
		{
			//Standardize the sample by subtracting the mean from the sample
			boost::numeric::ublas::matrix<double> standardizedSample(numDimensions, 1);
			for (unsigned int d = 0; d < numDimensions; ++d)
			{
				standardizedSample(d, 0) = sample[d] - mean.values[d];
			}

			//Now calculate the multivariate distance measure
			//distance = transpose(standardizedSample) * inverse(dataCovarianceMatrix) * standardizedSample
			
			//First calculate inverse(dataCovarianceMatrix) * standardized sample,
			//which is equivalent to solving dataCovarianceMatrix * x = standardizedSample
			boost::numeric::ublas::matrix<double> partialSolution = standardizedSample;
			boost::numeric::ublas::lu_substitute(dataCovarianceMatrix, dataPm, partialSolution);
			
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