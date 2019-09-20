class QMC5883L {
  public:
    void init();
    void reset();
    int  ready();
    void reconfig();

    int readHeading();
    int readRaw( int *x, int *y, int *z, int *t );

    void resetCalibration();

    void setSamplingRate( int rate );
    void setRange( int range );
    void setOversampling( int ovl );

  private:
    int xhigh, xlow;
    int yhigh, ylow;
    int addr;
    int mode;
    int rate;
    int range;
    int oversampling;
};
