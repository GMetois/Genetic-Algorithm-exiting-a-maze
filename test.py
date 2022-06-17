import shapely.geometry as sg
import matplotlib.pyplot as plt
import descartes

# create the circles with shapely
if __name__ == '__main__':
    a = sg.Point(-.5,0).buffer(1.)
    b = sg.Point(0.5,0).buffer(1.)

    point = sg.Point(0.2,0.2)
    left = a.difference(b)
    right = b.difference(a)
    middle = a.intersection(b)
    isIn = b.intersection(a)
    print(isIn)
    print(left.area)
    print(right.area)
    print(middle.area)

    ax = plt.gca()
    ax.add_patch(point)
    ax.add_patch(descartes.PolygonPatch(left, fc='b', ec='k', alpha=0.2))
    ax.add_patch(descartes.PolygonPatch(right, fc='r', ec='k', alpha=0.2))
    ax.add_patch(descartes.PolygonPatch(middle, fc='g', ec='k', alpha=0.2))

    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_aspect('equal')
    plt.show()