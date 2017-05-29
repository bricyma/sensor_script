class TestClass:
    a = 1

    def __init__(self):
        self.b = 2

    def display(self):
        print '1'


class TestClass2:
    c = 3

if __name__ == '__main__':
    check = TestClass()
    check.display()
    print check.a
    print check.b




