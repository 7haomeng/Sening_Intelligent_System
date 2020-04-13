# By Sean Lu, last edited at 04/03, 2020

import numpy as np

'''
Computing mean of given data
@in_param: in_data: (3, N) matrix
@out_param: mean: (3, 1) vector
'''
def compute_mean(in_data):
	shape = in_data.shape
	mean = np.mean(in_data, axis = 1)
	mean.shape = (shape[0], 1)
	return mean

# Numpy implementation of "Least-squares fitting of two 3-D point sets"	
def point_registration(source, target):
	assert source.shape[0] == 3
	assert target.shape[0] == 3
	assert source.shape[1] == target.shape[1]
	source_mean = compute_mean(source) # Eq. 4
	target_mean = compute_mean(target) # Eq. 5
	q = source - source_mean # Eq. 7
	q_ = target - target_mean # Eq. 8
	H = np.matmul(q, q_.T) # Eq. 11
	u, s, vh = np.linalg.svd(H) # Eq. 12
	R = np.matmul(vh.T, u.T) # Eq. 13
	T = target_mean - np.matmul(R, source_mean) # Eq. 10
	# Eq. 9, in MSE sense
	residual = q_ - np.matmul(R, q)
	mse = 0.0
	for i in range(residual.shape[1]):
		mse += np.linalg.norm(residual[:, i])
	mse /= residual.shape[1]
	print "Point sets registration MSE: %f" %mse
	return R, T
	
# Target transfomration matrix	
T = np.array([[0.28, 0.96, 0, 0.1], 
              [0.96, -0.28, 0, 0.4],
              [0, 0, -1, -0.3]])
# Source, in homogeneous form              
data_homo = np.array([[1, 2, 4, 0, 1, 3], 
                      [1, 3, 2, 1, 0, 1], 
                      [2, 1, 3, 1, 2, 1],
                      [1, 1, 1, 1, 1, 1]])
# Target                 
data_ = np.matmul(T, data_homo)
# Source
data = data_homo[0:3]

R, T = point_registration(data, data_)
print "R: \n", R
print "\nT: \n", T
# Add some random noise
print "=====================================\nWith corruption"
data_corrupted = data_ + np.random.normal(0, 0.01, data_.shape)
R, T = point_registration(data, data_corrupted)
print "R: \n", R
print "\nT: \n", T
