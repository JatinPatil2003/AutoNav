import { PlaceholderPage } from "../../../components/ui/PlaceholderPage";
import { useSelectedRobot } from "../../../app/robotSelection/useSelectedRobot";

export function NavigationPlaceholderPage() {
  const { selectedRobot } = useSelectedRobot();
  return (
    <PlaceholderPage
      title="Navigation"
      subtitle={selectedRobot ? `Robot: ${selectedRobot.displayName ?? selectedRobot.name}` : undefined}
    />
  );
}
