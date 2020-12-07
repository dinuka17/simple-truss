# -- Imports ------------------------------------------------------------------
import numpy as np


# -- Main class ---------------------------------------------------------------
class SimpleTruss:

    def __init__(self, name):
        """ Constructor
        """
        self.name = name
        self.members = {}
        self.load = None


    def __str__(self):
        """ String representation
        """
        return str(self.members)


    def add_member(self, near, far):
        """ This function adds a member to the truss given its near and far
        end coordinates. Note that each coordinate needs a boolean to indicate
        whether the node is a support or not. As such, the coordinates must be
        defined as a tuple: (x, y, bool). For example a member with coordinates
        (0,0) and (3,0) with the latter also being a support would be defined
        as:
           add_member((0, 0, False), (3, 0, True))
        """
        if len(self.members) == 0:
            self.members[1] = (
                (near[0], near[1], near[2], 1, 2),
                (far[0], far[1], far[2], 3, 4),
            )
        else:
            nodes = [i for sublist in self.members.values() for i in sublist]
            max_dof = max([i[4] for i in nodes])
            near_dof = None
            far_dof = None
            for n in nodes:
                if n[0] == near[0] and n[1] == near[1]:
                    near_dof = (n[3], n[4])
                if n[0] == far[0] and n[1] == far[1]:
                    far_dof = (n[3], n[4])
            if near_dof is None and far_dof is None:
                near_dof = (max_dof+1, max_dof+2)
                far_dof = (max_dof+3, max_dof+4)
            if near_dof is not None and far_dof is None:
                far_dof = (max_dof+1, max_dof+2)
            if near_dof is None and far_dof is not None:
                near_dof = (max_dof+1, max_dof+2)

            self.members[len(self.members) + 1] = (
                (near[0], near[1], near[2], near_dof[0], near_dof[1]),
                (far[0], far[1], far[2], far_dof[0], far_dof[1]),
            )


    def add_load(self, coords, load):
        """ This function adds a load at a truss joint. Input must be the
        coordinates of the node and the decomposed load in the x and y
        directions. For example, a 2kip downward load on joint with coordinates
        (0,0) will be defined as:
            add_load((0,0), (0, -2))
        """
        # Find the node and dofs
        dofs = None
        nodes = [i for sublist in self.members.values() for i in sublist]
        for n in nodes:
            if n[0] == coords[0] and n[1] == coords[1]:
                dofs = (n[3], n[4])
        self.load = (coords, load, dofs)


    def calc_length(self, xy1, xy2):
        """ Given the coordinates of two points, it will calculate the
        distance between the two points.
        """
        return np.linalg.norm(np.array(xy1) - np.array(xy2))


    def dir_cos(self, xy1, xy2):
        """ Returns the direction cosine for a member given point coordinates.
        """
        l = self.calc_length(xy1, xy2)
        return (xy2[0] - xy1[0])/l, (xy2[1] - xy1[1])/l


    def stiff_matrix(self, xy1, xy2, other_dofs=None):
        """ Returns the stiffness matrix for a member, given the coordinates of
        its near and far end nodes. Can optionally provide a tuple that includes
        other degrees of freedom in order to compile a matrix corresponding
        to the entire system.
        """
        length = self.calc_length(xy1, xy2)
        lx, ly = self.dir_cos(xy1, xy2)
        k = np.array(
            [
            [lx**2, lx*ly, -lx**2, -lx*ly],
            [lx*ly, ly**2, -lx*ly, -ly**2],
            [-lx**2, -lx*ly, lx**2, lx*ly],
            [-lx*ly, -ly**2, lx*ly, ly**2]
            ]
        ) / length

        if other_dofs:
            for i in other_dofs:
                k = np.insert(k, i-1, 0, axis=0)
                k = np.insert(k, i-1, 0, axis=1)

        return k


    def solve(self):
        """ This function uses all available data and returns the loads in the
        members and forces at the supports.
        """

        if self.load is None:
            raise ValueError ('Truss load has not been defined')

        nodes = [i for sublist in self.members.values() for i in sublist]
        max_dof = max([i[4] for i in nodes])

        k = np.zeros((max_dof, max_dof))
        for member in self.members.values():
            all_dofs = [i for i in range(1,max_dof+1)]
            member_dofs = [
                member[0][3], member[0][4], member[1][3], member[1][4]]
            rem_dofs = [i for i in all_dofs if i not in member_dofs]

            xy1 = (member[0][0], member[0][1])
            xy2 = (member[1][0], member[1][1])
            k += self.stiff_matrix(xy1, xy2, other_dofs=rem_dofs)

        # Displacement at load joint
        load_joint_k = k[self.load[2][0]-1:self.load[2][1],
                         self.load[2][0]-1:self.load[2][1]]
        d = np.linalg.inv(load_joint_k).dot([self.load[1][0], self.load[1][1]])

        # Reactions at supports
        reaction_k = k[2:, :2]
        q = reaction_k.dot(d)

        # forces in members
        forces = []
        for member in self.members.values():
            xy1 = (member[0][0], member[0][1])
            xy2 = (member[1][0], member[1][1])
            lx, ly = self.dir_cos(xy1, xy2)
            length = self.calc_length(xy1, xy2)
            q_m = (np.array([-lx, -ly, lx, ly])/length).dot([d[0], d[1], 0, 0])
            forces.append(q_m)

        return list(q), forces
