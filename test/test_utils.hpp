//-----------------------------------------------------------------------------
template <typename MsgCovType>
void fillMsgCovariance(MsgCovType & covariance, size_t start=0)
{
  for(size_t n=0;n<covariance.size();++n)
  {
    covariance[n]=n+start;
  }
}

//-----------------------------------------------------------------------------
template <typename EigenVectorType>
void fillEigenVector(EigenVectorType & vector, int start=0)
{
  for(int n=0;n<vector.rows();++n)
  {
    vector(n)=n+start;
  }
}

//-----------------------------------------------------------------------------
template <typename EigenCovType>
void fillEigenCovariance(EigenCovType & covariance, int start=0)
{
  for(int n=0;n<covariance.rows()*covariance.cols();++n)
  {
    covariance(n)=n+start;
  }
}

//-----------------------------------------------------------------------------
template <typename EigenType>
void isSame(const EigenType & obj1,const EigenType & obj2)
{
  for(int n=0;n<obj1.cols()*obj1.rows();++n)
  {
    EXPECT_DOUBLE_EQ(obj2(n), obj2(n));
  }
}
