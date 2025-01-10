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