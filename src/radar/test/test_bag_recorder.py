"""Tests for the guarded ARS408 bag recorder."""

import unittest

from radar.bag_recorder import (
    EXPECTED_TOPICS,
    duplicate_publisher_error,
    publishers_ready,
)


class BagRecorderGuardTest(unittest.TestCase):
    """Verify that recording starts only for one live radar publisher."""

    def test_one_publisher_per_topic_is_ready(self):
        counts = {topic: 1 for topic in EXPECTED_TOPICS}
        self.assertTrue(publishers_ready(counts))
        self.assertIsNone(duplicate_publisher_error(counts))

    def test_missing_publisher_is_not_ready(self):
        counts = {topic: 1 for topic in EXPECTED_TOPICS}
        counts["/ars408/status"] = 0
        self.assertFalse(publishers_ready(counts))
        self.assertIsNone(duplicate_publisher_error(counts))

    def test_duplicate_filter_or_player_is_rejected(self):
        counts = {topic: 1 for topic in EXPECTED_TOPICS}
        counts["/ars408/points"] = 2
        self.assertFalse(publishers_ready(counts))
        self.assertIn(
            "/ars408/points=2",
            duplicate_publisher_error(counts),
        )
