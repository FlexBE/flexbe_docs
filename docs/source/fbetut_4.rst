Developing Basic States
=======================

This section will go over a basic template and overview of developing a new state implementation.

1 Introduction
--------------

**States** define what should be done by a behavior.
They are grouped as statemachines, defining the control flow when the behavior is executed.
Therefore, each state declares a set of outcomes which represent possible results of execution.
Furthermore, data can be shared and modified during runtime between states.
This data is called *userdata* and stored as key-value-pairs.
Each state declares which keys it requires (input) and provides (output).

In short, a state is nothing else than a Python class that inherits from a common base class, the ``EventState`` class, and override a bunch of methods.
While only a few lines of code are required to implement a simple state, it is highly recommended to follow the conventions described here for ensuring code quality.
For example, although it is valid to declare multiple classes in a single file, please note that it is recommended to define only a single state per file and most tools, such as the *FlexBE WebUI*, will ignore additional ones.

Typically, implementing states is the only time where you manually write code when working with FlexBE.
Thus, please do it carefully, include documentation and tests (see below), and consider :ref:`sharing<Contributions>` your state implementations if they could be of interest for others as well.
Similarly, there already could be state implementations available providing what you need.
So check the `basic states <https://github.com/team-vigir/flexbe_behavior_engine/tree/master/flexbe_states/src/flexbe_states>`_ and existing `state repositories <https://github.com/FlexBE>`_ first.
Maybe, you do not even need to implement a new state.

1.1 Documentation
~~~~~~~~~~~~~~~~~

Documenting what your state does and how its interface works is an important part of implementing a new state.
Otherwise, people (including yourself after some time!) do not know how to use it or even re-implement it because its purpose is not clearly described.
It is a good approach to first specify the interface when developing a new state by writing the documentation and only afterwards, start implementing the documented specifications.

The documentation interface for a state is very simple and will be explained below.
Using this format allows the FlexBE user interface to parse the documentation and display it in the list of available states, the state properties, or during runtime to help the operator understand what the robot is doing.

The following example is a shortened version of the `flexbe_states/CalculationState <https://github.com/team-vigir/flexbe_behavior_engine/blob/master/flexbe_states/src/flexbe_states/calculation_state.py>`_ documentation on GitHub and contains all different parts:

.. code-block:: python

 class CalculationState(EventState):
    '''
    Implements a state that can perform a calculation based on userdata.
    calculation is a function which takes exactly one parameter, input_value from userdata,
    and its return value is stored in output_value after leaving the state.

    -- calculation  function	The function that performs the desired calculation.
                                It could be a private function (self.foo) manually defined in a behavior's source code
                                or a lambda function (e.g., lambda x: x^2, where x will be the input_value).

    ># input_value  object		Input to the calculation function.

    #> output_value object		The result of the calculation.

    <= done						Indicates completion of the calculation.
    '''

The documentation of a state follows the class declaration as a python docstring.
First, the purpose of the state itself is described.
The first sentence is considered to give an overview, the rest provides more details.
Afterwards, the four different types of the state interface are documented, typically grouped by their type and in the order given above.

The format is always given as (whitespace separated):

    * Interface type identifier
    * Name
    * Object type (not for outcomes)
    * Description

These are the available interface types and their identifiers:

+------------+-------------+--------------------------------------------+
| Type       | Identifier  | Purpose                                    |
+============+=============+============================================+
| Parameter  | --          | Argument to the state constructor,         |
|            |             | parametrizing the state instantiation.     |
+------------+-------------+--------------------------------------------+
| Input Key  | >#          | Runtime data required by this state.       |
+------------+-------------+--------------------------------------------+
| Output Key | #>          | Runtime data provided by this state.       |
+------------+-------------+--------------------------------------------+
| Outcome    | <=          | Possible result of execution.              |
+------------+-------------+--------------------------------------------+

1.2 State Tests
~~~~~~~~~~~~~~~

Similarly to documentation, well-defined state tests are an important part of implementing a new state and ensure that it works as documented.
Especially since python does not have a compiler, basic test cases are a highly recommended way to verify that the code is valid.

Although writing test cases is a topic many developers don't feel enthusiastic about, you really should do it.
With FlexBE, writing simple test cases is not harder than writing the documentation and as well, you wouldn't feel enthusiastic about restarting your behavior multiple times just because of trivial things like typos or mixed tab/space indentation.

Writing test cases is explained in a different tutorial: Writing State Tests Using flexbe_testing **(Coming Soon!)**.
After completing the current tutorial, feel free to continue with this one.

2 Implementation
----------------

States are implemented as python classes which inherit from ``EventState``.
Thus, do not forget to include the following import to your class definition:

.. code-block:: pythom

 from flexbe_core import EventState, Logger

The ``Logger`` class provides the FlexBE logging functionality.
It extends the standard rospy logger by sending logged messages to the operator and displaying it in the user interface.
Use this for printing important information or warnings, but do not print a large amount of text.

2.1 Functions
~~~~~~~~~~~~~

The ``EventState`` class provides a set of functions which can be implemented by a state definition.

**Constructor**

.. code-block:: python

 def __init__(self, target_time):
    super(ExampleState, self).__init__(outcomes = ['continue', 'failed'])

    self._target_time = rospy.Duration(target_time)
    self._start_time = None

The constructor of a state is called when the state is initialized, i.e., when the behavior is constructed before execution.
Here, variables are declared and proxies are initialized (see section below).

Most importantly in a constructor, the constructor of the superclass has to be called as shown in line 2 above.
This is where the interface of a state is defined.
The constructor of the superclass takes three arguments as list of strings:

+-------------+--------------+-----------------------------------------------------+
| outcomes    | *required*   | List of all outcomes to be returned by this state.  |
+-------------+--------------+-----------------------------------------------------+
| input_keys  | *optional*   | List of all userdata keys required by this state.   |
+-------------+--------------+-----------------------------------------------------+
| output_keys | *optional*   | List of all userdata keys provided by this state.   |
+-------------+--------------+-----------------------------------------------------+

Finally, parameters of the state are given by the arguments of its constructor, e.g., ``target_time`` in the example above.

**Execution Loop**

.. code-block:: python

 def execute(self, userdata):
    if rospy.Time.now() - self._start_time < self._target_time:
        return 'continue'

The execution loop is implemented by the execute function and called periodically (default: 10 hz) while the state is active.
It is supposed to evaluate conditions in order to check whether the state can return one of its outcomes.
Outcomes are returned as one of the strings defined as outcome in the constructor.
If no outcome is returned, e.g., in the example above because the condition is not fulfilled, the state will stay active.

The function has one argument: *userdata*.
This variable provides access to the input and output keys defined in the constructor.
Note that you can only read input keys and only write to output keys:

.. code-block:: console

 my_value = userdata.my_defined_input_key
 userdata.my_defined_output_key = my_value

**Events**

.. code-block:: python

 def on_enter(self, userdata):
        time_to_wait = rospy.Time.now() - self._start_time - self._target_time
        if time_to_wait > 0:
                Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)

The ``EventState`` superclass provides some further functions as callbacks to certain events.
An overview of all available events is provided in the tutorial The State Lifecycle **(Coming Soon!)**.
Most useful is the ``on_enter`` event, which is called once when a state becomes active.
Typically, it is used to set variables to their initial values or send action goals.

2.2 Proxies
~~~~~~~~~~~

Almost all of the time, your state implementation either publishes or subscribes to a topic, makes a service call (be careful when blocking execution), or acts as an action client (recommended, see in-depth tutorial Developing Actionlib States **(Coming Soon!)**).
In order to make this access more efficient when a larger amount of state instantiations refer to the same topics, FlexBE provides a collection of proxies.

You can import the proxies you need in your state by selecting from the following:

.. code-block:: python

 from flexbe_core.proxy import ProxyPublisher
 from flexbe_core.proxy import ProxySubscriberCached
 from flexbe_core.proxy import ProxyServiceCaller
 from flexbe_core.proxy import ProxyActionClient

All the proxies are instantiated by optionally passing them a dictionary of topics and their types to work with, for example:

.. code-block:: python

 self._sub = ProxySubscriberCached({'/a_topic': MsgType})

Of course, make sure to import the referred message type.
Furthermore, it is recommended to store the topic in a variable as you will need it again later.
The best place for instantiating a proxy with its topics of interest is the constructor of a state.
This way, a behavior will make sure that all of its required topics are available before it is started, which reduces the risk for runtime failure.

Using a proxy works the same way you would use a standard rospy publisher or subscriber, with one single difference.
They provide the same functions and expect the same arguments.
But in addition, the first argument passed to any function is the topic name you refer to, since a proxy manages multiple topics.

Furthermore, the proxy subscriber provides a caching mechanism in order to work better with the state execution loop instead of registering an asynchronous callback.
You can either access the last message received on the given topic (which typically is what you need in a state), or, for any topic separately, enable a message buffer which stores all messages not yet being processed in order to not lose any of them.
The following example shows how the proxy subscriber can be used in the execution loop of a state in order to check a condition:

.. code-block:: python

 if self._sub.has_msg(self._topic):
        msg = self._sub.get_last_msg(self._topic)
        # in case you want to make sure the same message is not processed twice:
        self._sub.remove_last_msg(self._topic)

And... That's it! You've completed all the basic tutorials. Feel free to continue with any :ref:`advanced tutorial<Advanced Tutorials>` of your choice or start using FlexBE in your own project.

