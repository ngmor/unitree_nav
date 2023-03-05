from typing import Text

from launch.condition import Condition
from launch.substitution import Substitution
from launch.launch_context import LaunchContext

class TernaryTextSubstitution(Substitution):
    """Substitution that returns the first text argument if the condition is true, otherwise it returns the second text argument."""
    def __init__(self, condition: Condition, true_text: Text, false_text: Text) -> None:
        """Create a TernaryTextSubstitution."""
        super().__init__()

        if not isinstance(true_text, Text):
            raise TypeError(
                "TernaryTextSubstitution expected Text object got '{}' instead.".format(type(true_text))
            )
        if not isinstance(false_text, Text):
            raise TypeError(
                "TernaryTextSubstitution expected Text object got '{}' instead.".format(type(false_text))
            )
        
        if not isinstance(condition, Condition):
            raise TypeError(
                "TernaryTextSubstitution expected Condition object got '{}' instead.".format(type(condition))
            )

        self.__true_text = true_text
        self.__false_text = false_text
        self.__condition = condition

    @property
    def true_text(self) -> Text:
        """Getter for true text."""
        return self.__true_text
    
    @property
    def false_text(self) -> Text:
        """Getter for false text."""
        return self.__false_text
    
    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"'{self.true_text}' if true, '{self.false_text}' if false"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string itself."""
        if self.__condition.evaluate(context):
            return self.true_text
        else:
            return self.false_text