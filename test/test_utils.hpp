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
template <typename MsgCovType, typename EigenCovType>
void isSame(const MsgCovType & msgCovariance,
            const EigenCovType &obsCovariance)
{
  for(size_t n=0;n<msgCovariance.size();++n)
  {
    EXPECT_DOUBLE_EQ(msgCovariance[n], obsCovariance(n));
  }
}

//-----------------------------------------------------------------------------
template <typename EigenCovType>
void isSame(const EigenCovType & obsCovariance1,
           const EigenCovType & obsCovariance2)
{
  for(int n=0;n<obsCovariance1.cols()*obsCovariance1.rows();++n)
  {
    EXPECT_DOUBLE_EQ(obsCovariance2(n), obsCovariance2(n));
  }
}
