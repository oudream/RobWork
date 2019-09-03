from rw import *

if __name__ == '__main__':
    rotd = Rotation3d(1,0,0,0,0,-1,0,1,0);
    rotf = Rotation3f(1,0,0,0,0,-1,0,1,0);
    
    print("Rotation double:");
    print(str(rotd));
    print("Rotation float:");
    print(str(rotf));
    print("Rotation inverse:");
    print(str(inverse(rotd)));
    print("Identity:");
    print(str(rotd*inverse(rotd)));