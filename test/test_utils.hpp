// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

//-----------------------------------------------------------------------------
template<typename MsgCovType>
void fillMsgCovariance(MsgCovType & covariance, size_t start = 0)
{
  for (size_t n = 0; n < covariance.size(); ++n) {
    covariance[n] = n + start;
  }
}

//-----------------------------------------------------------------------------
template<typename EigenVectorType>
void fillEigenVector(EigenVectorType & vector, int start = 0)
{
  for (int n = 0; n < vector.rows(); ++n) {
    vector(n) = n + start;
  }
}

//-----------------------------------------------------------------------------
template<typename EigenCovType>
void fillEigenCovariance(EigenCovType & covariance, int start = 0)
{
  for (int n = 0; n < covariance.rows() * covariance.cols(); ++n) {
    covariance(n) = n + start;
  }
}

//-----------------------------------------------------------------------------
template<typename EigenType>
void isSame(const EigenType & obj1, const EigenType & obj2)
{
  for (int n = 0; n < obj1.cols() * obj1.rows(); ++n) {
    EXPECT_DOUBLE_EQ(obj2(n), obj2(n));
  }
}


#endif  // TEST_UTILS_HPP_
