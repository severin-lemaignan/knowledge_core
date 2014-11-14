import functools

def memoize(obj):
    """
    Taken from https://wiki.python.org/moin/PythonDecoratorLibrary#Memoize
    """
    cache = obj.cache = {}

    @functools.wraps(obj)
    def memoizer(*args, **kwargs):
        key = str(args) + str(kwargs)
        if key not in cache:
            res = obj(*args, **kwargs)

            # never memoize undecided reasoning result: we want to re-evaluate everytime
            if res == "undecided":
                return res

            cache[key] = res
        return cache[key]
    return memoizer

