from lexium_agro.reproducibility import SeedContext


def test_namespaced_streams_are_repeatable_and_distinct() -> None:
    context = SeedContext(42)
    first = context.random("scene").random()
    assert first == context.random("scene").random()
    assert first != context.random("perception").random()


def test_derived_seed_is_uint64() -> None:
    derived = SeedContext(0).derive("evaluation")
    assert 0 <= derived < 2**64
