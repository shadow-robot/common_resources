/*
* Copyright (C) 2020 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
* Unauthorized copying of the content in this file, via any medium is strictly prohibited.
*/

#ifndef SR_UTILITIES_COMMON_PRIVATE_CPP_MOCK_LIB_H
#define SR_UTILITIES_COMMON_PRIVATE_CPP_MOCK_LIB_H

class CppMockLIb
{
public:
  /* IMPORTANT: If you want to obtain current tfs, make sure you pass the tf buffer
     to a tf listener outside of this class */
  CppMockLIb();
  ~CppMockLIb();
  int mock_value = 0;

};

#endif  // SR_UTILITIES_COMMON_PRIVATE_CPP_MOCK_LIB_H
