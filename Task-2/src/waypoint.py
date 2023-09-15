import numpy as np

class Rectangle:
    def __init__(self,X,Y,z,d0, d1):
        self.X = X
        self.Y = Y
        self.d0 = d0
        self.d1 = d1
        self.z = z

        #number of points in each side
        self.S1 = int(np.sqrt((X[0]-X[1])**2+(Y[0]-Y[1])**2)*d0)
        self.S2 = int(np.sqrt((X[1]-X[2])**2+(Y[1]-Y[2])**2)*d1)
        self.S3 = int(np.sqrt((X[2]-X[3])**2+(Y[2]-Y[3])**2)*d0)
        self.S4 = int(np.sqrt((X[3]-X[0])**2+(Y[3]-Y[0])**2)*d1)

        #slope and intercept of each side
        self.m = np.zeros(4)
        self.c = np.zeros(4)

    #compute slopes and intercepts from calculation of deviation
    def compute_slopes(self):
        if(self.X[0]!=self.X[1]):
            self.m[0] = (self.Y[0]-self.Y[1])/(self.X[0]-self.X[1])
            self.c[0] = -self.X[0]*self.m[0] + self.Y[0]
        else:
            self.m[0] = 'inf'
        if(self.X[1]!=self.X[2]):
            self.m[1] = (self.Y[1]-self.Y[2])/(self.X[1]-self.X[2])
            self.c[1] = -self.X[1]*self.m[0] + self.Y[1]
        else:
            self.m[1] = 'inf'
        self.m[2] = self.m[0]
        self.c[2] = -self.X[2]*self.m[0] + self.Y[2]
        self.m[3] = self.m[1]
        self.c[3] = -self.X[3]*self.m[0] + self.Y[3]

    #generate points for each side, and then combine them into one array of points for the rectangle
    def generate_points(self):
        self.X1 = np.linspace(self.X[0],self.X[1],num=self.S1,endpoint=False)
        self.X2 = np.linspace(self.X[1],self.X[2],num=self.S2,endpoint=False)
        self.X3 = np.linspace(self.X[2],self.X[3],num=self.S3,endpoint=False)
        self.X4 = np.linspace(self.X[3],self.X[0],num=self.S4,endpoint=True)
        self.Y1 = np.linspace(self.Y[0],self.Y[1],num=self.S1,endpoint=False)
        self.Y2 = np.linspace(self.Y[1],self.Y[2],num=self.S2,endpoint=False)
        self.Y3 = np.linspace(self.Y[2],self.Y[3],num=self.S3,endpoint=False)
        self.Y4 = np.linspace(self.Y[3],self.Y[0],num=self.S4,endpoint=True)
        X = np.concatenate((self.X1,self.X2,self.X3,self.X4))
        Y = np.concatenate((self.Y1,self.Y2,self.Y3,self.Y4))
        Z = np.zeros(len(X))
        Z.fill(self.z)
        XYZ = np.vstack((X,Y,Z))
        self.XYZ = XYZ.transpose()
        return self.XYZ


    #compute deviation from the rectangle for a given point (cx,cy,cz), where i is the index of the point in the rectangle
    def deviation(self,i,cx,cy,cz):
        if i<=self.S1:
            j = 0
        elif i<=self.S1+self.S2:
            j = 1
        elif i<=self.S1+self.S2+self.S3:
            j = 2
        else:
            j = 3
        if self.m[j]=='inf':
            dev = np.sqrt((cx-self.X[j])**2 + (self.z-cz)**2)
        else:
            dist = (cy - cx*self.m[j] - self.c[j])/np.sqrt(1+self.m[j]**2)
            dev = np.sqrt(dist**2 + (cz-self.z)**2)
        return dev