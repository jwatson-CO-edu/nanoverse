# Design Principles
* Take what varies and “encapsulate” it so it won’t affect the rest of your code.
* Program to an Interface, not to an implementation.
    - Program to a supertype
    - Delegate: Interface for each behavior, load a contained object that implements the interface
* Favor Composition over Inheritance
    - HAS-A can be better than IS-A
    - Remember the Banana Problem
* Strive for loosely-coupled designs between objects that interact: More flexible and resilient to change
* Classes should be open for extension, but closed for modification
* Depend upon abstractions. Do not depend upon concrete classes
    - No variable should hold a reference to a concrete class.
    - No class should derive from a concrete class.
* Principle of Least Knowledge: Talk only to your immediate friends
* Don't Call Us, We'll Call You
    - allow low-level components to hook themselves into a system, but the high-level components determine when they are needed, and how.
    - Dependency rot happens when you have a large number of vertical and lateral dependencies
* A class should have only one responsibility
    - Every responsibility of a class is an area of potential change
    - We say that a module or class has **high cohesion** when it is designed around a set of related functions

# Observer Pattern
* One-to-Many Relationships
* Loose Coupling: When two objects are loosely coupled, they can interact, but they typically have very little knowledge of each other.
* Subject: Publishes to all Observers
* Observer: Subscribes to Subject(s)
    * Observers should NOT depend on a specific notification order! (Think ROS!)

# Decorator Pattern
* Prevents class explosion
* Wraps around the object it decorates, Decorators can nest
* The outermost object recursively computes a result: The decorator adds its own behavior before and/or after delegating to the object it decorates to do the rest of the job.

# Factory Pattern
* Defines an interface for creating an object
* Factories handle the details of object creation, They encapsulate object creation by letting subclasses decide what objects to create
* Factories may have many clients, so we contain the details of object creation to a single class
* Clients should not actually depend on the concrete classes created by the factory!
* Abstract Factory
    - Factory that contains factories
    - May contain a factory for each superclass of object

# Singleton Pattern
* ensures a class has only one instance, and provides a global point of access to it. (Can use `enum`)
* Use a static class var to enforce one instantiation only!
* Making everything `static` may not be the answer!

# Command Pattern
* Encapsulates a request as an object, thereby letting you parameterize other objects with different requests, queue or log requests, and support undoable operations: Encapsulates method invocation, a DSL
* Receiver: The object operated on when the request is processed
* no other objects really know what actions get performed on what receiver; they just know that if they call the execute() method, their request will be serviced.
* lambda expressions to skip the step of creating concrete command objects
* We can undo a command if we logged the state before processing the last command
* A job queue supports thread-based execution of commands
* MacroCommands are a simple extension of the Command Pattern that allow multiple commands to be invoked. Likewise, MacroCommands can easily support `undo()`.

# Adapter Pattern 
* Can change the interface while reusing as much code as possible
* There are two forms of the Adapter Pattern: object and class adapters. Class adapters require multiple inheritance.

# Facade Pattern
* An adapter that simplifies the interface that has been adapted

# Template Method Pattern
* Think C++ Template Library
* Template Method Pattern defines the skeleton of an algorithm in a method, deferring some steps to subclasses. 
* Template Method lets subclasses redefine certain steps of an algorithm without changing the algorithm’s structure.
    - Keep abstract methods small in number
* Template Method implements the same algorithm on different targets, Strategy implements different algorithms for different actions at runtime
* Use abstract methods when your subclass MUST provide an implementation of the method or step in the algorithm. 
* Use hooks when that part of the algorithm is optional. With hooks, a subclass may choose to implement that hook, but it doesn’t have to.

# Iterator Pattern
* Get elements of the container, even if the containers are different in structure
* Implements an iterator interface: `has_next()` + `next_item()` + `remove_item()` (optional)
* See `java.util.Iterator`

# Composite Pattern