class Foo:
    def __init__(self, a):
        print("Foo initialized")
        self.name = "Foo method :/"
        self.a = a
        self.do_stuff()
         
    
    def do_stuff(self):
        print(self.a)

    def say_my_name(self):
        print(self.name)

class Bar(Foo):
    def __init__(self, b, a):
        print("bar initialized")
        self.b = b
        super().__init__(a)
        self.name = "Bar method :)"

    def do_stuff(self):
        print(self.b)
        print("this is the Bar method for do_stuff")

a = Bar("Marty", "Alex")
a.say_my_name()
