#/* Gray Thomas, UT Austin---NSTRF NNX15AQ33H---Summer 2018 */
""" tests the control_LMI module. """
from nose.tools import assert_equal, assert_is_not_none, raises, assert_true
# assert_raises, assert_true, assert_almost_equal,
# assert_almost_equal, assert_is_instance, raises
import numpy as np
import control
# import matplotlib.pyplot as plt
from state_space_objects import *
import nose

def test_clmi_import():
    import state_space_objects as clmi
    assert_is_not_none(clmi.__doc__)


def test_shouldpass():
    """ todo doc"""
    print('in test shouldpass. ')
    assert_equal(1, 1.0)

class TestClass(object):
    """ fake test class """
    def __init__(self):
        self.var = 1.0
    def test_class(self):
        """ fake test method """
        print('in test_class')
        assert_equal(1.0, self.var)
    @raises(AssertionError)
    def test_class_fail(self):
        """ fake test method, should fail with assertion error. """
        assert_equal([2.0, 1.0], [self.var, 1.0])

def setup_module(module):
    """ See when this is called with nosetests -s """
    print(module)
    print("does this first")

def test_control_structure():
    """ builds some simple control objects. """
    mat_a, mat_b, mat_c, mat_d =\
        np.array([[0, 1], [-1, -1]]), np.array([[0], [1]]),\
        np.array([[1, 0], [0, 1]]), np.zeros((2, 1))
    system = control.ss(mat_a, mat_b, mat_c, mat_d)
    sys = StateSpace(mat_a, mat_b, mat_c, mat_d)
    # sys.in0.rename("u")
    # sys.u
    print(system)
    print(dir(system))
    print(dir(control))
    # data, times = control.impulse(system)
    # plt.plot(times, data)
    # assert_true(False)

def test_assigning_names():
    pass

def test_appending_systems():
    pass

def test_tf2ss():
    pass

def get_generic_second_order():
    """ Return a second order integrator with an extra, backwards input. """
    a_matrix = np.array([[0, 1], [0, 0]])
    b_matrix = np.array([[0, 0], [-1, 1]])
    c_matrix = np.array([[1, 0]])
    d_matrix = np.array([[0, 0]])
    system = StateSpace(a_matrix, b_matrix, c_matrix, d_matrix)
    return system

def test_control_library_version():
    """ Tests the version of the control library."""
    assert_equal(control.__version__, "0.8.0")

def test_step_response():
    """ Demonstrate the control library's step function. """
    system = get_generic_second_order()
    time_vector = np.array([np.linspace(0, 1, 100), ])
    inputs_vector = np.array([
        np.linspace(0, 1, 100), # input 1
        np.linspace(0, 1, 100) # input 2
        ])
    initial_condition = np.array([
        [0.0], # state 1
        [0.0] # state 2
        ])
    time_result, output_result = control.step_response(system)
    time_result, output_result, state_result =\
        control.forced_response(system, time_vector,
                                inputs_vector, initial_condition)

def test_connect():
    """ Demonstrate the control library's connect function. """
    system = get_generic_second_order()
    system2 = get_generic_second_order()
    big_system = control.append(system, system2)
    print(system)
    print(big_system)

def test_new_connect():
    """ Demonstrate the control library's connect function. """

    system = get_generic_second_order()
    current = system.input_tags[0]
    current.name = "current"
    force = system.input_tags[1]

    system2 = get_generic_second_order()
    force2 = system2.input_tags[1]

    big_system = system.append(system2)
    assert big_system.output_tags[0] is system.output_tags[0]
    assert big_system.output_tags[1] is system2.output_tags[0]
    assert big_system.input_tags[1] is force
    assert big_system.input_tags[3] is force2
    new_output = (.5*big_system.get_output(system.output_tags[0])
        -1.2*big_system.get_output(system2.output_tags[0]))

    print(.5*big_system.get_output(system.output_tags[0])
        -1.2*big_system.get_output(system2.output_tags[0]))

    deflection, = big_system.new_output(new_output)
    ndeflection, = big_system.new_output(-new_output)

    # print(system)
    print(big_system)

    new_big_system = big_system.connect([(force, ndeflection),(force2, deflection)])

    print(new_big_system)

def test_connect_yellow_green():
    """ Test a particular example done on paper. """
    system = StateSpace("0,1;-2,-3","0;4","5,0","6")
    system.input_tags[0].name = "yellow"
    system.output_tags[0].name = "green"
    yellow, green = system.input_tags[0], system.output_tags[0]
    new_system = system.connect([(yellow, green)])
    assert_equal(new_system.input_tags[0].name, "yellow")
    assert_equal(new_system.output_tags[0].name, "green")
    np.testing.assert_allclose(new_system.A, np.array([[0, 1],[-6,-3]]))
    np.testing.assert_allclose(new_system.C, np.array([[-1, 0]]))
    np.testing.assert_allclose(new_system.D, np.array([[-6./5]]))
    np.testing.assert_allclose(new_system.B, np.array([[0],[-4./5]]))

def get_appended_system():
    """ Some test systems from pen and paperwork."""
    gy_system = StateSpace("0, 1;-2, -3", "0;4", "5,0", ".5")
    rb_system = StateSpace("0, 1;-7, -11", "0;13", "1,0", "17")
    ((yellow,), (green,)) = gy_system.tags
    ((red,), (blue,)) = rb_system.tags
    rgby_system = gy_system.append(rb_system)
    ((yellow_1, red_1), (green_1, blue_1)) = rgby_system.tags
    assert yellow_1 is yellow
    assert red_1 is red
    assert green_1 is green
    assert blue_1 is blue
    np.testing.assert_allclose(rgby_system.A, np.array([
        [0, 1, 0, 0],
        [-2, -3, 0, 0],
        [0, 0, 0, 1],
        [0, 0, -7, -11]]))
    np.testing.assert_allclose(rgby_system.B, np.array([
        [0, 0], [4, 0], [0, 0], [0, 13]]))
    np.testing.assert_allclose(rgby_system.C, np.array([
        [5, 0, 0, 0], [0, 0, 1, 0]]))
    np.testing.assert_allclose(rgby_system.D, np.array([
        [.5, 0], [0, 17]]))
    return rgby_system, yellow, red, green, blue

def assert_systems_close(system_1, system_2):
    """ Assertion for systems, to ensure similarity. """
    np.testing.assert_allclose(system_1.A, system_2.A)
    np.testing.assert_allclose(system_1.B, system_2.B)
    np.testing.assert_allclose(system_1.C, system_2.C)
    np.testing.assert_allclose(system_1.D, system_2.D)
    assert system_1.tags == system_2.tags

def test_rbyg_interconnections():
    """ Setup a combined system, and check interconnections against pen
    and paper. """
    system_0, yellow, red, green, blue = get_appended_system()
    system_1 = system_0.connect([(yellow, blue)])
    np.testing.assert_allclose(system_1.A, np.array([
        [0, 1, 0, 0],
        [-2, -3, 4, 0],
        [0, 0, 0, 1],
        [0, 0, -7, -11]]))
    np.testing.assert_allclose(system_1.B, np.array([
        [0, 0], [4, 68], [0, 0], [0, 13]]))
    np.testing.assert_allclose(system_1.C, np.array([
        [5, 0, .5, 0], [0, 0, 1, 0]]))
    np.testing.assert_allclose(system_1.D, np.array([
        [.5, 8.5], [0, 17]]))
    ((yellow_1, red_1), (green_1, blue_1)) = system_1.tags
    assert yellow_1 is yellow
    assert red_1 is red
    assert green_1 is green
    assert blue_1 is blue
    system_2 = system_0.connect([(red, green)])
    np.testing.assert_allclose(system_2.A, np.array([
        [0, 1, 0, 0],
        [-2, -3, 0, 0],
        [0, 0, 0, 1],
        [65, 0, -7, -11]]))
    np.testing.assert_allclose(system_2.B, np.array([
        [0, 0], [4, 0], [0, 0], [13./2., 13]]))
    np.testing.assert_allclose(system_2.C, np.array([
        [5, 0, 0, 0], [85, 0, 1, 0]]))
    np.testing.assert_allclose(system_2.D, np.array([
        [.5, 0], [8.5, 17]]))

    system_3 = system_0.connect([(red, green), (yellow, blue)])
    system_4 = system_0.connect([(yellow, blue), (red, green)])
    system_5 = system_2.connect([(yellow, blue)])
    system_6 = system_1.connect([(red, green)])
    assert_systems_close(system_3, system_4)
    assert_systems_close(system_3, system_5)
    assert_systems_close(system_3, system_6)

    # is this true in cases with more agressive D matrices?
    system_7 = system_0.connect([
        (red, green), (red, blue), (red, blue), (yellow, blue), (yellow, green)
        ])
    system_8 = system_0.connect([
        (red, blue), (yellow, blue), (yellow, green), (red, green), (red, blue)
        ])
    system_9 = system_0.connect([(red, blue)]).connect([(yellow, blue)]).connect(
        [(yellow, green)]).connect([(red, green)]).connect([(red, blue)])
    assert_systems_close(system_7, system_8)
    raises(AssertionError)(lambda: assert_systems_close(system_7, system_9))()

    # looks like it's not. Order matters in general, but order within one
    # connect operation doesn't matter.

def test_adding_velocity_filters():
    """ Tests the conecpt that adding a velocity filter is simple. """

    def get_sys(alpha=50.0, p_gain=100.0, d_gain=10.0):
        """ setup a simple feedback with velocity filtering. """
        sys = get_generic_second_order()
        ((incurrent, inforce), (outpos,)) = sys.tags
        low_pass = StateSpace(np.array([[-alpha]]), np.array([[alpha]]),\
            np.array([[-alpha]]), np.array([[alpha]]))
        low_pass = lp_diff_2(alpha)
        sys = sys.inline(outpos, low_pass)
        outvel = low_pass.output_tags[0]
        control_matrix = np.array([[p_gain, d_gain]])
        outcontrol, = sys.new_output(
            control_matrix.dot(np.vstack([
                sys.get_output(outpos),
                sys.get_output(outvel)]))
            )
        sys = sys.connect([(incurrent, outcontrol)])
        return sys
    assert (np.linalg.eig(get_sys(alpha=5000.0).A)[0] < 0).all()
    assert (np.linalg.eig(get_sys(alpha=50.).A)[0] < 0).all()
    assert not (np.linalg.eig(get_sys(alpha=10.).A)[0] < 0).all()

def test_cleaned_inputs():
    """ tests cleaning inputs with the trivial self-list"""
    sys = get_generic_second_order()
    sys.cleaned_inputs(sys.input_tags)

def test_get_input():
    """ tests basic getting of input."""
    sys = get_generic_second_order()
    sys.get_input(sys.input_tags[1])

def teardown_module(module):
    """ See when this is called with nosetests -s """
    print(module)
    # plt.show()
    print("does this last")
# print(platform)

if __name__ == '__main__':
    nose.main()
