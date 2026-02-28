import matplotlib.pyplot as plt
import numpy as np

# Fixing random state for reproducibility
np.random.seed(19680801)


# def randrange(n, vmin, vmax):
#     """
#     Helper function to make an array of random numbers having shape (n, )
#     with each number distributed Uniform(vmin, vmax).
#     """
#     return (vmax - vmin)*np.random.rand(n) + vmin

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# n = 100

# # For each set of style and range settings, plot n random points in the box
# # defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
# for m, zlow, zhigh in [('o', -50, -25), ('^', -30, -5)]:
#     xs = randrange(n, 23, 32)
#     ys = randrange(n, 0, 100)
#     zs = randrange(n, zlow, zhigh)
#     ax.scatter(xs, ys, zs, marker=m)

# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

# plt.show()

fpab = pd.read_csv('gps_testAtoB.csv')
fpca = pd.read_csv('gps_testCtoA.csv')
fpbc = pd.read_csv('gps_testBtoC.csv')

def convarr(fp):
    return fp[['timestamp', 'latitude', 'longitude','altitude']].to_numpy()


def lat_lon_to_cartesian(lat, lon, altitude):
    # Convert latitude and longitude to Cartesian coordinates (x, y, z)

    R = 6371  # Earth radius in kilometers
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    x = (R + altitude) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (R + altitude) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (R + altitude) * np.sin(lat_rad)

    return x, y, z

def convert_to_cartesian(latitudes, longitudes, altitudes):
    cartesian_coords = []
    for lat, lon, alt in zip(latitudes, longitudes, altitudes):
        x, y, z = lat_lon_to_cartesian(lat, lon, alt)
        cartesian_coords.append((x, y, z))
    return cartesian_coords

if __name__ == "__main__":
    # Example usage
    latitudes = [43.66093241666667, 43.6611142, 43.66181253333333]
    longitudes = [-79.39500986666667, -79.39477455, -79.39416395]
    altitudes = [73, 67, 69] 

    cartesian_coordinates = convert_to_cartesian(latitudes, longitudes, altitudes)
    print(cartesian_coordinates)
    