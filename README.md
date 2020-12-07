# Simple Truss Analysis Program

*This is a demo project for "CE-UY 3013 Computing In Civil Engineering". It*
*demonstrates the expected structure and some functionality. It is* ***not***
*indicative of the expected scope. Meaning, your project's scope should be*
*wider with more features. Good luck.*

---

This program calculates the forces in the members and at the supports of a two
dimensional truss. The joints or nodes are located at specific points in space
at fixed distance and are connected together by the members or elements to form
the framework.

Assumptions:
* members are considered prismatic having a constant cross-section over their length
* linear elastic behavior
* load is applied at the joints
* the support joint must be constrained of translational movement

Inputs:
* start & end coordinates of each member
* vertical and/or horizontal load at joint with coordinates (x,y)

Outputs:
* forces in members
* forces at supports


## Setup

In order to use the program, you have to clone/download this repository,
navigate to the local directory and create a virtual environment with:

```
$ python3 -m venv venv
```

Then, activate the virtual environment:

```
For Linux/Mac OS:
$ source venv/bin/activate

For Windows:
> venv\Scripts\activate
```

Finally, install the required libraries for this program with:

```
$ pip install -r requirements.txt
```


## How to use the program

<img src="https://storage.googleapis.com/nm-static/computing_maloof2_20200927.png" alt="nyu_comp_f20" style="max-height:100px">

Here is how we can analyze the truss above.

First instantiate a new object of ``SimpleTruss``:

```python
>>> truss = SimpleTruss('My first truss')
```

Next, we can add the members:

```python
>>> truss.add_member((0, 0, False), (3, 0, True))
>>> truss.add_member((0, 0, False), (3, 4, True))
```

Let's add a load:

```python
>>> truss.add_load((0,0), (0, -2))
```

Finally we can analyze the truss and print the results:

```python
>>> print(truss.solve())
```
