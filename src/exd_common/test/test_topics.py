from exd_common import topics


def test_topic_names_are_namespaced_and_unique():
    names = [v for k, v in vars(topics).items() if k.isupper() and not k.startswith('FRAME_')]
    assert names, 'no topics defined'
    assert all(n.startswith('/exd/') for n in names)
    assert len(names) == len(set(names)), 'duplicate topic name'
