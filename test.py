from source import SimpleTruss

truss = SimpleTruss('My first truss')
truss.add_member((0, 0, False), (3, 0, True))
truss.add_member((0, 0, False), (3, 4, True))
truss.add_load((0,0), (0, -2))

print(truss)
print(truss.solve())


print('\n')


truss2 = SimpleTruss('My second truss')
truss2.add_member((4, 4, False), (0, 0, True))
truss2.add_member((4, 4, False), (4, 0, True))
truss2.add_member((4, 4, False), (7, 0, True))
truss2.add_load((4, 4), (-500, 0))

print(truss2)
print(truss2.solve())
