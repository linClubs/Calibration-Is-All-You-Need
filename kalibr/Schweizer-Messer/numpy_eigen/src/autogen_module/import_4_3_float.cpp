// This file automatically generated by create_export_module.py
#define NO_IMPORT_ARRAY 

#include <NumpyEigenConverter.hpp>

#include <boost/cstdint.hpp>


void import_4_3_float()
{
	NumpyEigenConverter<Eigen::Matrix< float, 4, 3 > >::register_converter();
}

