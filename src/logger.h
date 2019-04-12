#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>

#define MATRIX_NAME "[MATRIX]"
#ifdef LOGGER_DEBUG
  #define MATRIX_DEBUG(name, arg){\
              std::cout<<MATRIX_NAME<<name<<std::endl<<arg<<std::endl; \
  }
#else
  #define MATRIX_DEBUG(name, arg)
#endif

#endif /* LOGGER_H_ */