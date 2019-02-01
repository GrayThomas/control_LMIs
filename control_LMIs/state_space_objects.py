#/* Gray Thomas, UT Austin---NSTRF NNX15AQ33H---Summer 2018 */
""" Feedback Control Systems Linear Matrix Inequality Utilites Module"""
import control as ctrl
import numpy as np

class IO(object):
    """ An input or output port, attached to a system. """

    def __init__(self, system, index=0, is_input=True, name=None):
        self.system = system
        self.index = index
        self.is_input = is_input
        if name is None:
            name = ("in%d" if is_input else "out%d")%index
        self.name = name

        # too much syntactic sugar. Makes the linter complain:
        # if hasattr(self.system, self.name):
        #     raise Exception()
        # setattr(self.system, self.name, self)

    def rename(self, new_name):
        """ renames the IO, and updates system attr. """
        delattr(self.system, self.name)
        self.name = new_name
        setattr(self.system, self.name, self)


class IO(object):
    """ A conch-like object, used to keep track of IO indexes. """

    def __init__(self, name=None):
        self.name = name


def _index(taglist, tag):
    for i, t in enumerate(taglist):
        if t is tag:
            return i
    raise ValueError("Tag not found"+tag.name)

class StateSpace(ctrl.StateSpace):
    """ An extension of control state space systems, with nameable ports.
    See the test cases in test_state_space_objects.py for usage. """

    def __init__(self, mat_a, mat_b, mat_c, mat_d):
        super().__init__(mat_a, mat_b, mat_c, mat_d)
        self.set_tags([IO() for row in self.B.T], [IO() for row in self.C])

    def set_tags(self, input_tags, output_tags):
        self.input_tags = list(input_tags)
        self.output_tags = list(output_tags)
        self.tags = (self.input_tags, self.output_tags)

    def copy(self):
        """ A copy function
        to preserve the original behavior of connect if desired.
        """
        ret = StateSpace(self.A, self.B, self.C, self.D)
        ret.set_tags(self.input_tags, self.output_tags)
        return ret

    def append(self, other):
        tmp = super().append(other)
        ret = StateSpace(tmp.A, tmp.B, tmp.C, tmp.D)
        # doesn't require ret.set_tags, since replacement is elementwise.
        for i in range(self.inputs):
            ret.input_tags[i] = self.input_tags[i]
        for i in range(self.outputs):
            ret.output_tags[i] = self.output_tags[i]
        if isinstance(other, StateSpace):
            for i in range(other.inputs):
                ret.input_tags[i+self.inputs] = other.input_tags[i]
            for i in range(other.outputs):
                ret.output_tags[i+self.outputs] = other.output_tags[i]
        return ret

    def new_output(self, c_d_matrix):
        """ Adds an output, defined by combined C and D row(s)."""
        self.C = np.vstack([self.C, c_d_matrix[:, :self.states]])
        self.D = np.vstack([self.D, c_d_matrix[:, self.states:]])
        super().__init__(self.A, self.B, self.C, self.D)
        new_tags = [IO() for row in c_d_matrix]
        self.output_tags.extend(new_tags)
        return new_tags

    def new_input(self, b_d_matrix):
        """ Adds an input, defined by combined B and D column(s)."""
        self.B = np.hstack([self.B, b_d_matrix[:self.states, :]])
        self.D = np.hstack([self.D, b_d_matrix[self.states:, :]])
        super().__init__(self.A, self.B, self.C, self.D)
        new_tags = [IO() for row in b_d_matrix.T]
        self.input_tags.extend(new_tags)
        return new_tags


    def get_output(self, tag):
        """ Get the combined C and D row corresponding to a tag."""
        ndx = _index(self.output_tags, tag)
        return np.hstack([self.C[[ndx], :], self.D[[ndx], :]])

    def get_input(self, tag):
        """ Get the combined B and D column corresponding to a tag."""
        ndx = _index(self.input_tags, tag)
        return np.vstack([self.B[:, [ndx]], self.D[:, [ndx]]])

    def connect(self, list_of_io):
        """ tag based interconnection function. """
        interconnection_matrix = []
        for tag_in, tag_out in list_of_io:
            interconnection_matrix.append([
                _index(self.input_tags, tag_in)+1,
                _index(self.output_tags, tag_out)+1])
        interconnection_matrix = np.array(interconnection_matrix)

        ret_ss = ctrl.connect(self, interconnection_matrix,
                     range(1, self.inputs+1), range(1, self.outputs+1))
        ret = StateSpace(ret_ss.A, ret_ss.B, ret_ss.C, ret_ss.D)
        ret.set_tags(self.input_tags, self.output_tags)
        return ret

    def cleaned_outputs(self, output_tags):
        """ return a copy of self with re-organized outputs."""
        c_d_matrixes = []
        for tag in output_tags:
            c_d_matrixes.append(self.get_output(tag))
        c_d_matrix = np.vstack(c_d_matrixes)
        ret = StateSpace(self.A, self.B, c_d_matrix[:, :self.states],
                         c_d_matrix[:, self.states:])
        ret.set_tags(self.input_tags, output_tags)
        return ret

    def cleaned_inputs(self, input_tags):
        """ return a copy of self with re-organized inputs."""
        b_d_matrixes = []
        for tag in input_tags:
            b_d_matrixes.append(self.get_input(tag))
        b_d_matrix = np.hstack(b_d_matrixes)
        ret = StateSpace(self.A, b_d_matrix[:self.states, :],
                         self.C, b_d_matrix[self.states:, :])
        ret.set_tags(input_tags, self.output_tags)
        return ret

    def inline(self, tag, system):
        """ Adds a system to an input or output. """
        sys = self.append(system)
        if tag in self.input_tags:
            return sys.connect([(tag, system.output_tags[0])])
        elif tag in self.output_tags:
            return sys.connect([(system.input_tags[0], tag)])
        raise ValueError("Tag not found"+tag.name)

def lp_diff_1(alpha):
    """ Simple first order low pass differentiator. """
    return StateSpace(np.array([[-alpha]]), np.array([[alpha]]),\
        np.array([[-alpha]]), np.array([[alpha]]))

def lp_diff_2(alpha):
    """ Simple stacked first order low pass differentiator. """
    return StateSpace(
        np.array([
            [-alpha, alpha],
            [0, -alpha]]),
        np.array([[0], [alpha]]),
        np.array([[-alpha, alpha]]),
        np.array([[0]]))
