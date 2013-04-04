void rCoordTransformTest() {
    Point a;
    a.x = 1;
    a.y = 2;
    Point b;
    b.x = 1;
    b.y = 1;
    Pose B;
    B.x = 2;
    B.y = 1;
    B.theta = PI / 2.0;
    Point bb = rCoordTransform(a, B);
    cout << b << endl;
    cout << bb << endl;
}
